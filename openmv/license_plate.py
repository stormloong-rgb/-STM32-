# ============ OpenMV 车牌识别 ============
import sensor
import image
import time
import display
from pyb import millis, UART
import os
try:
    import gc
except ImportError:
    gc = None

# ============ 参数配置 ============
UART_BAUDRATE = 115200
uart = UART(3, UART_BAUDRATE)

# 识别结果去抖参数
# 省份若用两位代码（CN_<CODE>.pgm → CODE），则归一化后为 8 位：HA + A + 5 位，如 豫A6GV02 -> HAA6GV02
MIN_PLATE_LEN = 8
# ====== 当前画质下的稳识别参数包（低清容错）======
# 目标：减少 no_province_hit，同时保持“结构合法才发送”
STABLE_FRAMES_REQUIRED = 6      # 连续稳定帧数（低清场景下稍放宽）
SEND_INTERVAL_MS = 1000         # 两次发送最小间隔（ms）
VOTE_WINDOW = 9                 # 多帧投票窗口（兼顾稳定与响应）
# 无真实省份模板时仅用 A–Z 做第 1 格会乱跳，默认禁止串口发送（可改 True 调试）
UART_REQUIRE_REAL_PROVINCE = True
# True：不扫描整库，只加载「豫(CN_HA)」省份模板。
# 你的当前测试牌固定为“豫V.280UU”，开启后可显著减少“川/甘/桂”等误省份。
PROVINCE_LOAD_MINIMAL_YU_ONLY = True

# ====== 定帧识别/降误识别（模板匹配更吃“好帧”） ======
RECOGNIZE_EVERY_N_FRAMES = 2        # 低清下增加采样机会
PLATE_RECT_STABLE_FRAMES = 3        # 绿框连续稳定帧数后才允许识别
RECT_STABLE_MAX_DX = 8
RECT_STABLE_MAX_DY = 6
RECT_STABLE_MAX_DW = 14
RECT_STABLE_MAX_DH = 10
SHARP_STDEV_MIN = 24               # 低清容错：适度放宽清晰度门限

# 绿框（整牌区域）稳健参数：防止右侧远处噪声把 x_max 拉爆
PLATE_MAX_PIXEL_W = 210       # 整牌外接矩形最大宽度（像素）
PLATE_PAD_X = 10              # 绿框横向扩展，降低“未完全包含”概率
PLATE_PAD_Y = 4               # 绿框纵向扩展
PLATE_MAX_LEFT_JUMP = 6       # 单帧允许的最大左移，抑制绿框向左漂
GAP_OUTLIER_MULT = 2.8        # 相邻 blob 间距 > 中位数*此值视为离群（删端点）
MIN_GAP_FOR_OUTLIER = 22    # 离群判定间距下限（像素）
PLATE_SMOOTH = 0.35           # 绿框指数平滑系数（越大越跟手，越小越稳）

# 蓝牌「字母·后五位」之间的分隔点：不做字模识别，仅占水平宽度，避免点挤占相邻 ROI
PLATE_DOT_GAP_MIN = 5         # 分隔点最小占位（像素），略大以免第 2 格压到点
PLATE_DOT_GAP_MAX = 14
PLATE_DOT_GAP_PER_CHAR = 0.38  # 相对预估单字宽 cw0，略减给字母格让路

# 同一行 blob 保留数量：蓝牌常见「汉字 + 发牌字母 + 5 位」共 7 块；原先固定保留最右 6 个会丢掉最左「豫」
PLATE_BLOB_KEEP_MAX = 7

# 第一格（汉字省简称）宽度：蓝牌上单字接近「方」，再紧一档避免右侧蓝底吃进字母/分隔点
PROVINCE_W_FRAC = 0.135
PROVINCE_W_REL_H = 0.78
PROVINCE_W_MIN_PX = 12
PROVINCE_W_MAX_PX = 28
PROVINCE_ROI_MAX_ASPECT_H = 0.92  # 省份格宽不超过 字高*系数（与红框高一致量级）
PLATE_LEFT_TRIM_PX = 6           # 低清下进一步保留左侧省份笔画
PLATE_RIGHT_TRIM_PX = 4

# 省份模板路径
PROVINCE_PATH = '/library_province'
ALPHANUMERIC_PATH = '/library_alphanumeric'

# 省份 code <-> 汉字（匹配模板名/串口ASCII输出用）
PROVINCE_CODE_TO_HAN = {
    "BJ": "京", "TJ": "津", "HE": "冀", "SX": "晋", "NM": "蒙",
    "LN": "辽", "JL": "吉", "HL": "黑", "SH": "沪", "JS": "苏",
    "ZJ": "浙", "AH": "皖", "FJ": "闽", "JX": "赣", "SD": "鲁",
    "HA": "豫", "HB": "鄂", "HN": "湘", "GD": "粤", "GX": "桂",
    "HI": "琼", "CQ": "渝", "SC": "川", "GZ": "贵", "YN": "云",
    "XZ": "藏", "SN": "陕", "GS": "甘", "QH": "青", "NX": "宁",
    "XJ": "新",
}
PROVINCE_HAN_TO_CODE = {v: k for k, v in PROVINCE_CODE_TO_HAN.items()}
def get_province_code_filenames():
    # 不直接依赖全局常量，避免未同步版本出现 NameError
    return tuple(["CN_%s.pgm" % k for k in PROVINCE_CODE_TO_HAN.keys()])

# 匹配参数
MATCHING_RATE = 0.80
MATCHING_RATE_MIN = 0.55
# 省份字（汉字）比字母数字更容易因模糊失配，单独放宽
PROVINCE_MATCHING_RATE = 0.66
PROVINCE_MATCHING_RATE_MIN = 0.38

# 蓝色车牌定位（仿照“示例”思路：先用颜色稳定位，再分割字符）
# OpenMV 的 LAB 阈值范围：L[0..100], A[-128..127], B[-128..127]
# 该阈值需按现场光照微调；先给一个偏宽的蓝色范围
BLUE_PLATE_THRESHOLDS = [
    (0, 70, -20, 40, -90, -15),  # 蓝牌（宽松）
]
BLUE_PIXELS_THRESHOLD = 650
BLUE_AREA_THRESHOLD = 650
BLUE_ASPECT_MIN = 2.2
BLUE_ASPECT_MAX = 6.5
BLUE_MERGE = True
BLUE_MARGIN = 8
PLATE_EXPAND_X = 10
PLATE_EXPAND_Y = 6
# 当前场景下蓝定位会引入抖动与异常，改回稳定的亮度/边缘定位
USE_BLUE_LOCATOR = False
# 蓝框几何约束（启用蓝色定位时生效）
BLUE_MIN_W = 70
BLUE_MIN_H = 18
BLUE_MAX_JUMP_X = 26
BLUE_MAX_JUMP_Y = 16
MIN_BLUE_HITS_IN_PLATE = 1

# ====== “示例”风格：跳变点/投影分割（替换等宽切格） ======
# True：按车牌ROI中线的黑白跳变点自动找字符边界；更像示例工程的“跳变点分割”
USE_SAMPLE_STYLE_SEGMENTATION = True
SEG_SCAN_BAND = 2              # 中线上下各取几行做投票，抗噪
SEG_BIN_OFFSET = 16            # 低清场景下白字对比下降，降低偏置保留细笔画
SEG_MIN_RUN_W = 3              # 放宽最小前景宽度，避免细字被误删
SEG_MERGE_GAP = 4              # 合并近邻断裂笔画
SEG_DOT_MAX_W = 12             # 对分隔点宽度放宽，避免误吞相邻字符
SEG_EXPECT_CHARS = 7           # 省+字母+5位（点不算字符）

# ====== 测试牌专用收敛（关闭=真实识别模式）======
TEST_PLATE_MODE = False
# 测试牌模式下禁用“样式分割”，改用稳定等宽切格（当前画面下更稳）
TEST_FORCE_FIXED_SEGMENTATION = True
# 测试牌模式下的字符匹配阈值（低清容错）
TEST_MATCHING_RATE = 0.72
TEST_MATCHING_RATE_MIN = 0.42
TEST_RAW_VOTE_WINDOW = 15
TEST_EXPECT_PLATES_ASCII = ("HAVH8X96", "HAV280UU")  # 连续测试两块牌
# 当前测试目标（单独测试某一块牌时，把这里改成对应值）
TEST_ACTIVE_TARGET_ASCII = "HAV280UU"
ENABLE_TARGET_SWITCH_CMD = False # 关闭手动切换目标
TEST_UART_EXACT_ONLY = False
TEST_FORCE_TARGET_IF_CLOSE = False
TEST_TARGET_MIN_MATCH = 5      # 与目标串同位至少命中N位则强制收敛
TEST_SECOND_PASS_MATCH = False # 避免二次低阈值匹配在某些帧触发异常
TEST_PLATE_X_SHIFT_RIGHT = 12  # 测试牌模式下将绿框整体右移，修正左偏
TEST_TARGET_SWITCH_MARGIN = 1   # 新目标证据领先门限（连续切牌时更灵敏）
TEST_MIN_OBS_FOR_BIAS = 1       # 放宽：只要有少量后5位证据就可启用偏置
TEST_SWITCH_BY_POS3_HINT = False # 关闭按位提示切目标
TEST_DUP_FORCE_SWITCH_FRAMES = 35  # 连续dup达到阈值后强制切换目标

# 部署模式建议关LCD，可减少显示链路引发的中断/卡顿
USE_LCD = False

# ============ 初始化 ============
sensor.reset()
sensor.set_pixformat(sensor.RGB565)
sensor.set_framesize(sensor.QVGA)
sensor.set_windowing((320, 172))
sensor.set_contrast(2)

clock = time.clock()

# 预分配图像缓冲区（必须在加载大量字模前分配，避免 MemoryError allocating 55KB）
if gc is not None:
    try:
        gc.collect()
    except Exception:
        pass

img_gray = image.Image(320, 172, sensor.GRAYSCALE)
img_gray2 = image.Image(320, 172, sensor.GRAYSCALE)

# 七格统一 28x45，降低峰值内存（省份 ROI 经 draw_image 缩放后进缓冲，find_template 仍可用）
def _alloc_char_targets():
    targets = []
    for w, h in ((28, 45), (24, 40)):
        try:
            targets = []
            for _ in range(7):
                targets.append(image.Image(w, h, sensor.GRAYSCALE))
            if w < 28:
                print("Char targets using fallback size:", w, h)
            return targets
        except MemoryError as me:
            print("Char target alloc failed", w, h, repr(me))
            targets = []
    raise MemoryError("char targets")

img_targets = _alloc_char_targets()

# ============ 加载模板 ============
def _sort_filenames_utf8(files):
    """
    MicroPython 对含中文文件名的 list 做 sorted() 可能抛 UnicodeError；
    用 UTF-8 字节序排序，顺序稳定且不与控制台编码冲突。
    """
    out = list(files)
    try:
        out.sort(key=lambda fn: fn.encode("utf-8") if isinstance(fn, str) else bytes(fn))
    except Exception:
        try:
            out.sort()
        except Exception:
            pass
    return out


def _safe_print_listdir(path, files):
    """
    OpenMV 串口控制台对 UTF-8 中文文件名支持差，直接 print(files) 会 UnicodeError。
    只打印数量与纯 ASCII 文件名；其余用 '*' 占位计数。
    """
    try:
        try:
            n = len(files)
        except Exception:
            return
        ascii_names = []
        non_ascii = 0
        for f in files:
            try:
                if all(ord(c) < 128 for c in f):
                    ascii_names.append(f)
                else:
                    non_ascii += 1
            except Exception:
                non_ascii += 1
        print("Files in", path, "count:", n, "ascii:", len(ascii_names), "non_ascii:", non_ascii)
        if ascii_names:
            print(" ascii names:", ascii_names)
    except UnicodeError:
        print("Files in", path, "(skipped listing names, UnicodeError)")


def load_templates(path, ascii_manifest=None):
    """
    加载模板文件索引，返回(文件路径列表, 名称列表)。
    注意：为避免 RAM 爆掉，这里不再 image.Image() 全量加载模板！
    ascii_manifest: 若 listdir 抛 UnicodeError，则只尝试这些纯 ASCII 文件名（如 province_yu.pgm）。
    """
    templates = []
    names = []
    try:
        print("Trying path:", path)
        files = []
        try:
            files = os.listdir(path)
        except UnicodeError:
            print("listdir UnicodeError, fallback manifest:", ascii_manifest)
            files = list(ascii_manifest) if ascii_manifest else []
        _safe_print_listdir(path, files)
        files = _sort_filenames_utf8(files)
        for f in files:
            if not (f.endswith('.pgm') or f.endswith('.png')):
                continue
            fp = path + '/' + f
            try:
                # 只保存路径，匹配时按需加载
                templates.append(fp)
                names.append(f[:-4])
            except Exception as fe:
                print("Skip bad template:", repr(fe))
        print("Loaded", len(templates), "templates from", path)
    except Exception as e:
        print("Error loading templates from", path, ":", repr(e))
    return templates, names

def _file_exists(fp):
    try:
        os.stat(fp)
        return True
    except Exception:
        return False


# 模板按需加载缓存（LRU，避免重复读 SD）
# 如果太小：每次 find_template 都会从 SD 反复读 pgm，FPS 会很低（你现在的 0.8FPS 就是典型症状）
# 如果太大：可能占用过多 RAM。这里取一个折中，并在 warmup 时遇到 MemoryError 自动停。
TEMPLATE_CACHE_MAX = 48
_tmpl_cache = {}
_tmpl_cache_order = []

def get_template_image(fp):
    img = _tmpl_cache.get(fp, None)
    if img is not None:
        return img
    # 读入并缓存
    img = image.Image(fp)
    _tmpl_cache[fp] = img
    _tmpl_cache_order.append(fp)
    if len(_tmpl_cache_order) > TEMPLATE_CACHE_MAX:
        old = _tmpl_cache_order.pop(0)
        try:
            del _tmpl_cache[old]
        except Exception:
            pass
    return img


def match_province_in_target(tgt_img):
    """在给定目标图里匹配省份模板，返回(命中bool, 名称或None)。"""
    if len(province_templates) == 0:
        return False, None
    for j, tmpl in enumerate(province_templates):
        if j >= len(province_names):
            continue
        try:
            _timg = get_template_image(tmpl)
            r = tgt_img.find_template(
                _timg, PROVINCE_MATCHING_RATE, step=1, search=image.SEARCH_EX
            )
            if not r:
                r = tgt_img.find_template(
                    _timg, PROVINCE_MATCHING_RATE_MIN, step=1, search=image.SEARCH_EX
                )
        except Exception:
            r = None
        if r:
            return True, province_names[j]
    return False, None


TEMPLATE_WARMUP = True          # 启动时尽量把模板预热进缓存，提高 FPS
WARMUP_ALNUM_MAX = 36           # 0-9 + A-Z
WARMUP_PROVINCE_MAX = 31        # CN_<CODE>.pgm 数量（你的目录是 31）

def warm_template_cache(paths, max_to_load):
    """
    尽量把模板加载进缓存，提高运行时 FPS。
    遇到 MemoryError 会自动停止并收缩缓存上限，避免再次炸内存。
    """
    global TEMPLATE_CACHE_MAX
    loaded = 0
    for fp in paths:
        if loaded >= max_to_load:
            break
        try:
            _ = get_template_image(fp)
            loaded += 1
            if gc is not None and (loaded % 6) == 0:
                try:
                    gc.collect()
                except Exception:
                    pass
        except MemoryError:
            # 内存不够就停：把上限收缩到当前已缓存数量，避免继续尝试导致抖动/崩溃
            TEMPLATE_CACHE_MAX = len(_tmpl_cache_order)
            print("Template warmup stopped by MemoryError; cache_max=", TEMPLATE_CACHE_MAX)
            break
        except Exception as e:
            # 单个模板坏了就跳过
            print("Template warmup skip:", fp, repr(e))
            continue
    print("Template warmup loaded:", loaded, "cache_size=", len(_tmpl_cache_order))


def province_slot_width_px(usable_w_px, inner_h_px):
    """汉字省简称格宽度：字高比例、整牌宽度比例与硬上限取最小再夹紧。"""
    pw_h = int(inner_h_px * PROVINCE_W_REL_H)
    pw_f = int(usable_w_px * PROVINCE_W_FRAC)
    pw = min(PROVINCE_W_MAX_PX, pw_h, pw_f)
    return max(PROVINCE_W_MIN_PX, pw)


def _province_stem_to_label(stem):
    """
    将省份字模文件名 stem 映射为“汉字省份”：
    - CN_<CODE>.pgm -> 对应汉字（如 CN_HA -> 豫）
    - 旧命名 province_yu -> 豫
    """
    if stem == "province_yu":
        return "豫"
    if stem.startswith("CN_") and len(stem) >= 5:
        code = stem[3:5]
        return PROVINCE_CODE_TO_HAN.get(code, stem)
    return stem


def detect_plate_rect_blue(img):
    """
    用蓝色阈值定位车牌大致矩形，返回 (x,y,w,h) 或 None。
    参考“示例”的思路：先找蓝牌区域，再进行字符分割。
    """
    try:
        blobs = img.find_blobs(
            BLUE_PLATE_THRESHOLDS,
            pixels_threshold=BLUE_PIXELS_THRESHOLD,
            area_threshold=BLUE_AREA_THRESHOLD,
            merge=BLUE_MERGE,
            margin=BLUE_MARGIN,
        )
    except Exception as _be:
        # 有些固件/帧上 merge 可能触发异常，降级一次无 merge
        try:
            blobs = img.find_blobs(
                BLUE_PLATE_THRESHOLDS,
                pixels_threshold=BLUE_PIXELS_THRESHOLD,
                area_threshold=BLUE_AREA_THRESHOLD,
                merge=False,
                margin=1,
            )
        except Exception as _be2:
            print("blue find_blobs:", repr(_be), "fallback:", repr(_be2))
            return None

    if not blobs:
        return None

    best = None
    best_area = 0
    for b in blobs:
        try:
            w = b.w()
            h = b.h()
            if h <= 0 or w <= 0:
                continue
            ar = w / h
            if ar < BLUE_ASPECT_MIN or ar > BLUE_ASPECT_MAX:
                continue
            a = b.area()
            if a > best_area:
                best_area = a
                best = b
        except Exception:
            continue

    if best is None:
        return None

    x = max(0, best.x() - PLATE_EXPAND_X)
    y = max(0, best.y() - PLATE_EXPAND_Y)
    w = min(320 - x, best.w() + PLATE_EXPAND_X * 2)
    h = min(172 - y, best.h() + PLATE_EXPAND_Y * 2)
    if w <= 0 or h <= 0:
        return None
    if w < BLUE_MIN_W or h < BLUE_MIN_H:
        return None
    return (x, y, w, h)


def plate_rect_blue_ok(img, rect):
    """对候选绿框做蓝色命中门控，避免非蓝牌背景进入字符识别。"""
    try:
        x, y, w, h = rect
        if w <= 0 or h <= 0:
            return False
        hits = img.find_blobs(
            BLUE_PLATE_THRESHOLDS,
            roi=(x, y, w, h),
            pixels_threshold=120,
            area_threshold=120,
            merge=False,
            margin=1,
        )
        return (hits is not None) and (len(hits) >= MIN_BLUE_HITS_IN_PLATE)
    except Exception:
        return False


def rect_is_stable(prev_rect, cur_rect):
    if prev_rect is None or cur_rect is None:
        return False
    px, py, pw, ph = prev_rect
    cx, cy, cw, ch = cur_rect
    return (
        abs(cx - px) <= RECT_STABLE_MAX_DX
        and abs(cy - py) <= RECT_STABLE_MAX_DY
        and abs(cw - pw) <= RECT_STABLE_MAX_DW
        and abs(ch - ph) <= RECT_STABLE_MAX_DH
    )


def sharp_enough(gray_img, rect):
    """用 ROI 灰度标准差做清晰度/纹理门控（反光发白、运动模糊时 stdev 会明显下降）。"""
    try:
        x, y, w, h = rect
        if w <= 0 or h <= 0:
            return False
        st = gray_img.get_statistics(roi=(x, y, w, h))
        return int(st.stdev()) >= SHARP_STDEV_MIN
    except Exception:
        return False


def _clamp_roi(x, y, w, h):
    x = max(0, min(319, int(x)))
    y = max(0, min(171, int(y)))
    w = max(1, min(320 - x, int(w)))
    h = max(1, min(172 - y, int(h)))
    return (x, y, w, h)


def segment_chars_like_sample(img_gray_src, plate_rect):
    """
    完全仿“示例”分割思想（跳变点/边界）：
    - ROI 内先用均值阈值二值化（白字=1，蓝底/暗=0）
    - 做每列白像素计数表 col_sum（相当于示例的“跳变点统计表”）
    - 用阈值把 col_sum 转成 0/1，再找 0->1 / 1->0 的边界点（k1..k8）
    - 得到 7 个字符框（省份+字母+后五位）；分隔点表现为很窄的“谷”，天然被当作边界间隙跳过
    """
    px, py, pw, ph = plate_rect
    px, py, pw, ph = _clamp_roi(px, py, pw, ph)
    if pw < 70 or ph < 18:
        return None

    # 二值阈值：ROI均值 + offset
    try:
        st = img_gray_src.get_statistics(roi=(px, py, pw, ph))
        thr = int(st.mean()) + SEG_BIN_OFFSET
    except Exception:
        thr = 140

    # 纵向统计范围（尽量避开上下边框）
    y1 = py + 2
    y2 = py + ph - 3
    if y2 <= y1:
        return None
    hscan = y2 - y1 + 1

    # 每列白像素计数 col_sum
    col_sum = [0] * pw
    for dx in range(pw):
        x = px + dx
        s = 0
        for yy in range(y1, y2 + 1):
            try:
                if img_gray_src.get_pixel(x, yy) >= thr:
                    s += 1
            except Exception:
                pass
        col_sum[dx] = s

    # 把 col_sum 转成 0/1（列内白像素超过阈值视作字符列）
    # 经验阈值：高度的 18%（白字笔画较细，设低一些）
    on_thr = max(2, int(hscan * 0.18))
    col_on = [1 if v >= on_thr else 0 for v in col_sum]

    # 平滑：小孔洞填平（gap<=SEG_MERGE_GAP）
    i = 0
    while i < pw:
        if col_on[i] == 0:
            j = i
            while j < pw and col_on[j] == 0:
                j += 1
            if 0 < j - i <= SEG_MERGE_GAP:
                # 若两侧都是1，填洞
                if i > 0 and j < pw and col_on[i - 1] == 1 and col_on[j] == 1:
                    for k in range(i, j):
                        col_on[k] = 1
            i = j
        else:
            i += 1

    # 找边界：runs of 1
    runs = []
    in_run = False
    rs = 0
    for idx, v in enumerate(col_on):
        if v == 1 and not in_run:
            in_run = True
            rs = idx
        elif v == 0 and in_run:
            re = idx - 1
            if re - rs + 1 >= SEG_MIN_RUN_W:
                runs.append([rs, re])
            in_run = False
    if in_run:
        re = pw - 1
        if re - rs + 1 >= SEG_MIN_RUN_W:
            runs.append([rs, re])

    if not runs:
        return None

    # 合并非常近的 run
    merged = []
    cur = runs[0]
    for r in runs[1:]:
        if r[0] - cur[1] <= SEG_MERGE_GAP:
            cur[1] = r[1]
        else:
            merged.append(cur)
            cur = r
    merged.append(cur)

    # 去掉很窄的 run（分隔点/铆钉），示例里点会形成窄跳变
    segments = []
    for a, b in merged:
        w = b - a + 1
        if w <= SEG_DOT_MAX_W:
            continue
        segments.append([a, b])

    if len(segments) < 5:
        segments = merged

    # 目标 7 段：多则取宽度最大的 7 段（示例本质也是取有效字符区）
    if len(segments) > SEG_EXPECT_CHARS:
        segments = sorted(segments, key=lambda r: (r[1] - r[0]), reverse=True)[:SEG_EXPECT_CHARS]
        segments.sort(key=lambda r: r[0])

    if len(segments) != SEG_EXPECT_CHARS:
        return None

    # 输出字符框
    boxes = []
    by = py + 2
    bh = max(18, ph - 4)
    for a, b in segments:
        x = px + a
        w = b - a + 1
        boxes.append([x, by, max(4, w), bh])
    return boxes


def check_storage():
    """检查存储设备"""
    try:
        files = os.listdir('/')
        print("Storage contents:", files)
        return files
    except Exception as e:
        print("Storage check error:", e)
        return []

print("Checking storage...")
storage_info = check_storage()

# 数字字母模板路径 (你改过文件夹名的话，这里多给一些候选)
paths_to_try = [
    '/sdcard/templates_lp',
    '/sdcard/openmv/library_alphanumeric',
    '/sdcard/openmv/library-alphanumeric',
    '/sdcard/openmv/alphanumeric',
    '/sdcard/openmv/alnum',
    '/sdcard/library_alphanumeric',
    '/sdcard/library-alphanumeric',
    '/sdcard/alphanumeric',
    '/sdcard/alnum',
    '/sdcard/library_alnum',
    '/sdcard/templates_alphanumeric',
    '/flash/library_alphanumeric',
]

print("Loading alphanumeric templates...")
alphanumeric_templates = []
alphanumeric_names = []
for path in paths_to_try:
    t, n = load_templates(path)
    if t:
        alphanumeric_templates = t
        alphanumeric_names = n
        break

print(f"Alphanumeric templates: {len(alphanumeric_templates)}")

# 尝试多个可能路径 - 省份（CN_<CODE>.pgm 命名也可放在这些目录）
paths_to_try = [
    '/sdcard/templates_lp_cn',
    '/sdcard/openmv/library_province',
    '/sdcard/openmv/library-province',
    '/sdcard/openmv/province',
    '/sdcard/openmv/province_cn',
    '/sdcard/openmv/library_province_cn',
    '/sdcard/library_province',
    '/sdcard/library-province',
    '/sdcard/province',
    '/sdcard/province_cn',
    '/sdcard/library_province_cn',
    '/sdcard/templates_province',
    '/flash/library_province',
    '/flash/province',
]

province_templates = []
province_names = []
province_loaded_from_dir = False

# ASCII 文件名优先：FAT/串口下比「豫.pgm」更不易出问题（仓库内已提供 province_yu.pgm 副本）
_SINGLE_PROVINCE_CANDIDATES = [
    "/sdcard/templates_lp_cn/CN_HA.pgm",
    "/sdcard/library_province/CN_HA.pgm",
    "/sdcard/CN_HA.pgm",
    "/sdcard/library_province/province_yu.pgm",
    "/sdcard/province_yu.pgm",
    "/sdcard/library_province/豫.pgm",
    "/sdcard/豫.pgm",
    "/flash/templates_lp_cn/CN_HA.pgm",
    "/flash/library_province/CN_HA.pgm",
    "/flash/library_province/province_yu.pgm",
    "/flash/library_province/豫.pgm",
]

print("Loading province templates...")
if PROVINCE_LOAD_MINIMAL_YU_ONLY:
    print("Province: minimal YU-only load (PROVINCE_LOAD_MINIMAL_YU_ONLY=True)")
    province_single_han_loaded = False
    for _p in _SINGLE_PROVINCE_CANDIDATES:
        try:
            if not _file_exists(_p):
                continue
            province_templates.append(_p)  # 路径
            province_names.append("豫")
            province_single_han_loaded = True
            print("Loaded single province template:", _p)
            break
        except Exception:
            pass
    if not province_templates:
        print("Minimal province load failed; copy openmv/library_province/province_yu.pgm to SD.")
        print("WARNING: Using alphanumeric as province fallback...")
        province_templates = alphanumeric_templates[:26] if len(alphanumeric_templates) >= 26 else alphanumeric_templates
        province_names = alphanumeric_names[:26] if len(alphanumeric_names) >= 26 else alphanumeric_names
else:
    province_single_han_loaded = False
    _prov_manifest = get_province_code_filenames() + ("province_yu.pgm",)
    for path in paths_to_try:
        t, n = load_templates(path, ascii_manifest=_prov_manifest)
        if t:
            province_templates = t
            province_names = [_province_stem_to_label(x) for x in n]
            province_loaded_from_dir = True
            if "豫" in province_names:
                province_single_han_loaded = True
            break

    if not province_templates:
        print("WARNING: No province templates loaded! Using alphanumeric as fallback...")
        province_templates = alphanumeric_templates[:26] if len(alphanumeric_templates) >= 26 else alphanumeric_templates
        province_names = alphanumeric_names[:26] if len(alphanumeric_names) >= 26 else alphanumeric_names

    # 已在目录/manifest 里加载「豫」则勿再 insert 同一路径，否则会 Province templates: 2 且首格语义错乱
    if "豫" not in province_names:
        for _p in _SINGLE_PROVINCE_CANDIDATES:
            try:
                if not _file_exists(_p):
                    continue
                province_templates.insert(0, _p)  # 路径
                province_names.insert(0, "豫")
                province_single_han_loaded = True
                print("Loaded single province template:", _p)
                break
            except Exception:
                pass

# 仅从 SD 加载到「豫」字模、未成功扫目录时：去掉 A–Z 回退，避免第 1 格被当成普通字母乱匹配
if province_single_han_loaded and not province_loaded_from_dir:
    if len(province_templates) > 0 and len(province_names) > 0:
        province_templates = [province_templates[0]]
        province_names = [province_names[0]]
        print("Province: using single Han slot only (trimmed A-Z fallback).")

province_ok_for_uart = province_loaded_from_dir or province_single_han_loaded
if UART_REQUIRE_REAL_PROVINCE and not province_ok_for_uart:
    print("UART: disabled until real province template(s) on SD (see 豫.pgm / library_province).")

print(f"Province templates: {len(province_templates)}")

# 预热模板缓存：避免每次匹配都从 SD 读模板导致 FPS 过低
if TEMPLATE_WARMUP:
    try:
        if gc is not None:
            try:
                gc.collect()
            except Exception:
                pass
        # 先预热字母数字模板（最常用）
        if alphanumeric_templates:
            warm_template_cache(alphanumeric_templates, WARMUP_ALNUM_MAX)
        # 再预热省份模板（数量也不多）
        if province_templates:
            warm_template_cache(province_templates, WARMUP_PROVINCE_MAX)
    except Exception as _we:
        print("Template warmup error:", repr(_we))

# 初始化结果存储
license_number = [' '] * 7
last_norm_plate = ""
stable_count = 0
last_sent_plate = ""
last_sent_tick = 0
plate_history = []
test_raw_history = []
ACTIVE_TEST_TARGET = TEST_ACTIVE_TARGET_ASCII if TEST_ACTIVE_TARGET_ASCII in TEST_EXPECT_PLATES_ASCII else TEST_EXPECT_PLATES_ASCII[0]
test_dup_streak = 0
last_plate_rect = None  # (px, py, pw, ph) 绿框时间平滑
last_blue_rect = None
frame_id = 0
rect_stable_count = 0
_prev_rect_for_stable = None

def _median_simple(lst):
    if not lst:
        return 0
    lst = sorted(lst)
    return lst[len(lst) // 2]

def filter_row_blobs_remove_spacing_outliers(blobs):
    """
    按 x 排序，反复去掉与邻居间距异常大的端点 blob（常见为右侧远处噪声）。
    """
    if len(blobs) <= 2:
        return sorted(blobs, key=lambda b: b.x())
    blobs = sorted(blobs, key=lambda b: b.x())
    while len(blobs) >= 3:
        gaps = []
        for i in range(1, len(blobs)):
            gaps.append(blobs[i].cx() - blobs[i - 1].cx())
        mg = _median_simple(gaps)
        thresh = max(MIN_GAP_FOR_OUTLIER, int(mg * GAP_OUTLIER_MULT))
        if gaps[-1] >= thresh:
            blobs.pop(-1)
            continue
        if gaps[0] >= thresh:
            blobs.pop(0)
            continue
        break
    return blobs

# 汉字省份 -> ASCII 两位 code：已在文件顶部定义 PROVINCE_HAN_TO_CODE

def normalize_plate(raw):
    """去空格/点号，中文省份转 ASCII（两位省份代码），其余仅保留英文数字"""
    s = ""
    for ch in raw:
        if ch in " .\t-\u3000":
            continue
        if ch in PROVINCE_HAN_TO_CODE:
            s += PROVINCE_HAN_TO_CODE[ch]
            continue
        o = ord(ch)
        if o > 127:
            # 其他汉字：无映射则跳过（避免乱码进串口）
            continue
        if ('0' <= ch <= '9') or ('A' <= ch <= 'Z') or ('a' <= ch <= 'z'):
            s += ch.upper()
    return s

def correct_plate_by_position(plate):
    """
    仅对第 1 位（省份槽误识成数字时）做易混淆字母替换。
    不含 1->I：无牌/噪点时常出「11111」，会把整窗投票污染成 I1111。
    """
    if len(plate) == 0:
        return plate

    chars = list(plate)

    if len(chars) < 5:
        return plate

    digit_to_alpha = {
        '0': 'O',
        '2': 'Z',
        '5': 'S',
        '8': 'B',
    }
    ch0 = chars[0]
    if ch0 in digit_to_alpha:
        chars[0] = digit_to_alpha[ch0]

    return ''.join(chars)


def _noise_string_for_vote(s):
    """无真实车牌时易出现的单调串，不参与多帧投票，避免拖垮 vote。"""
    if s is None or len(s) < MIN_PLATE_LEN:
        return False
    t = s.strip()
    if len(t) < MIN_PLATE_LEN:
        return False
    if len(set(t)) == 1:
        return True
    if len(t) >= MIN_PLATE_LEN and t.replace("1", "") == "":
        return True
    return False

def plausible_plate_ascii(s):
    """
    小型汽车常见：
    - 7 位 ASCII（省份映射为 1 字符）
    - 或 8 位 ASCII（省份映射为 2 位代码，如 HA），第 3 位为发牌机关字母
    用于过滤明显乱码，减少误发串口。
    """
    if s is None or len(s) not in (7, 8):
        return False
    if len(s) == 7:
        if not ("A" <= s[1] <= "Z"):
            return False
        idxs = (0, 2, 3, 4, 5, 6)
        for i in idxs:
            c = s[i]
            if not (("0" <= c <= "9") or ("A" <= c <= "Z")):
                return False
    else:
        # 8 位：省份两位代码 + 字母 + 后五位
        if not ("A" <= s[0] <= "Z" and "A" <= s[1] <= "Z"):
            return False
        if not ("A" <= s[2] <= "Z"):
            return False
        for i in (3, 4, 5, 6, 7):
            c = s[i]
            if not (("0" <= c <= "9") or ("A" <= c <= "Z")):
                return False
    return True


def vote_plate(candidates, prefer_len=7):
    """
    多帧按位投票：
    - 先选择出现最多的长度（与 prefer_len 接近时优先 prefer_len，贴近蓝牌 7 位）
    - 再对每一位字符做多数投票
    """
    if len(candidates) == 0:
        return ""

    # 统计长度频次
    len_count = {}
    for s in candidates:
        ln = len(s)
        len_count[ln] = len_count.get(ln, 0) + 1

    best_len = max(len_count, key=lambda k: len_count[k])
    max_v = len_count[best_len]
    if prefer_len in len_count and len_count[prefer_len] >= max(1, max_v - 1):
        best_len = prefer_len

    same_len = [s for s in candidates if len(s) == best_len]
    if len(same_len) == 0:
        return ""

    out = []
    for idx in range(best_len):
        char_count = {}
        for s in same_len:
            ch = s[idx]
            char_count[ch] = char_count.get(ch, 0) + 1
        best_ch = max(char_count, key=lambda k: char_count[k])
        out.append(best_ch)
    return ''.join(out)


def _vote_char_with_allowed(raw_list, idx, allowed, prefer=None):
    cnt = {}
    for s in raw_list:
        if idx >= len(s):
            continue
        ch = s[idx]
        if ch in allowed:
            cnt[ch] = cnt.get(ch, 0) + 1
    if not cnt:
        return prefer if (prefer is not None and prefer in allowed) else ' '
    best = max(cnt.values())
    cands = [k for k, v in cnt.items() if v == best]
    if (prefer is not None) and (prefer in cands):
        return prefer
    return cands[0]


def build_test_plate_from_raw_history(raw_list):
    """
    将多帧 7 位原始字符槽（含空格）按位投票，拼成测试牌格式：
    豫 + V + 3位数字 + 2位字母
    """
    if len(raw_list) == 0:
        return ""
    global ACTIVE_TEST_TARGET

    def _target_evidence(target):
        # 统计“后5位”被观察到且与目标一致的证据
        score = 0
        obs = 0
        if len(target) < 8:
            return 0, 0
        suf = target[3:8]
        for s in raw_list:
            if len(s) < 7:
                continue
            for k in range(5):
                ch = s[2 + k]
                if ch == ' ':
                    continue
                obs += 1
                if ch == suf[k]:
                    score += 1
        return score, obs

    out = [' '] * 7
    out[0] = "豫"
    out[1] = "V"

    # 先做一轮不带偏置的粗投票，再根据证据选择/维持当前目标
    for pos in range(5):
        out[2 + pos] = _vote_char_with_allowed(raw_list, 2 + pos, "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ")

    coarse = normalize_plate(''.join(out))
    # 若后5位可用观测太少，不做目标偏置，避免长期被某一目标“吸住”
    _obs_total = 0
    for s in raw_list:
        if len(s) < 7:
            continue
        for k in range(5):
            if s[2 + k] != ' ':
                _obs_total += 1
    if _obs_total < TEST_MIN_OBS_FOR_BIAS:
        # 仅识别到“豫V/HAV”时，使用当前活动目标补全，避免长期停在 HAV
        if coarse == "HAV" and len(ACTIVE_TEST_TARGET) == 8:
            return ACTIVE_TEST_TARGET
        return coarse

    # 证据打分 + 滞回切换（避免在两目标间频繁跳）
    best_t = ACTIVE_TEST_TARGET
    best_score = -1
    second_score = -1
    for t in TEST_EXPECT_PLATES_ASCII:
        s1 = same_pos_score(coarse, t)
        s2, _ = _target_evidence(t)
        s = s1 + s2
        if s > best_score:
            second_score = best_score
            best_score = s
            best_t = t
        elif s > second_score:
            second_score = s
    if (best_score - second_score) >= TEST_TARGET_SWITCH_MARGIN:
        ACTIVE_TEST_TARGET = best_t

    tgt = ACTIVE_TEST_TARGET
    suffix = tgt[3:8] if (tgt is not None and len(tgt) >= 8) else ""
    for pos in range(5):
        pref = suffix[pos] if len(suffix) > pos else None
        if pref is not None and ("0" <= pref <= "9"):
            allow = "0123456789"
        elif pref is not None and ("A" <= pref <= "Z"):
            allow = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        else:
            allow = "0123456789ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        out[2 + pos] = _vote_char_with_allowed(raw_list, 2 + pos, allow, prefer=pref)
    return normalize_plate(''.join(out))


def same_pos_score(a, b):
    if (a is None) or (b is None):
        return 0
    n = min(len(a), len(b))
    s = 0
    for i in range(n):
        if a[i] == b[i]:
            s += 1
    return s


def _target_by_pos3_kind(want_digit):
    """按目标牌第3位(点后首位)是数字/字母，返回对应目标串。"""
    for t in TEST_EXPECT_PLATES_ASCII:
        if len(t) < 4:
            continue
        c = t[3]
        if want_digit and ("0" <= c <= "9"):
            return t
        if (not want_digit) and ("A" <= c <= "Z"):
            return t
    return None


def _extract_target_cmd(buf):
    """
    解析串口切换命令：
    - @TARGET=HAV280UU
    - @TARGET=HAVH8X96
    - @T0 (切到 TEST_EXPECT_PLATES_ASCII[0])
    - @T1 (切到 TEST_EXPECT_PLATES_ASCII[1])
    """
    if buf is None:
        return None
    try:
        s = buf.decode("ascii", "ignore").upper()
    except Exception:
        return None
    if len(s) == 0:
        return None
    if "@T0" in s:
        return TEST_EXPECT_PLATES_ASCII[0] if len(TEST_EXPECT_PLATES_ASCII) > 0 else None
    if "@T1" in s:
        return TEST_EXPECT_PLATES_ASCII[1] if len(TEST_EXPECT_PLATES_ASCII) > 1 else None
    k = "@TARGET="
    p = s.find(k)
    if p >= 0:
        t = s[p + len(k): p + len(k) + 8]
        if t in TEST_EXPECT_PLATES_ASCII:
            return t
    return None

invert = False

# 显示初始化（无 SPI 屏时避免未捕获异常；部分固件此处会 MemoryError）
lcd = None
if USE_LCD:
    try:
        lcd = display.SPIDisplay()
    except Exception as _disp_e:
        print("Display disabled:", repr(_disp_e))
        lcd = None

# ============ 主循环 ============
print("Starting main loop...")
while True:
    try:
        clock.tick()
        if TEST_PLATE_MODE and ENABLE_TARGET_SWITCH_CMD:
            try:
                if uart.any():
                    _rx = uart.read()
                    _sw = _extract_target_cmd(_rx)
                    if _sw is not None and _sw != ACTIVE_TEST_TARGET:
                        ACTIVE_TEST_TARGET = _sw
                        test_raw_history = []
                        plate_history = []
                        last_norm_plate = ""
                        stable_count = 0
                        print("Test target switched by uart cmd:", ACTIVE_TEST_TARGET)
            except Exception as _rx_e:
                print("uart cmd read:", repr(_rx_e))
        img = sensor.snapshot()
        
        img_gray.draw_image(img, 0, 0)
        if invert:
            img_gray.invert()
        
        img_gray2.draw_image(img_gray, 0, 0)
        img_gray.laplacian(3)
        img_gray.gamma_corr(gamma=1.2, contrast=25)

        # ---------- 车牌定位（优先蓝色定位，失败再回退旧法） ----------
        plate_rect = None
        if USE_BLUE_LOCATOR:
            try:
                cand_rect = detect_plate_rect_blue(img)
            except Exception as _blue_e:
                print("blue locator:", repr(_blue_e))
                cand_rect = None
            if cand_rect is not None:
                if last_blue_rect is not None:
                    bx, by, bw, bh = last_blue_rect
                    cx, cy, cw, ch = cand_rect
                    # 蓝框跳变过大时忽略本帧蓝定位，避免抖动拉偏分割
                    if abs(cx - bx) <= BLUE_MAX_JUMP_X and abs(cy - by) <= BLUE_MAX_JUMP_Y:
                        plate_rect = cand_rect
                        last_blue_rect = cand_rect
                    else:
                        plate_rect = None
                else:
                    plate_rect = cand_rect
                    last_blue_rect = cand_rect

        if plate_rect is not None:
            plate_x, plate_y, plate_w, plate_h = plate_rect
            h_avg = max(18, int(plate_h / 1.3))
        else:
            try:
                blobs = img_gray.find_blobs(
                    [(2, 255)], x_stride=4, y_stride=2,
                    pixels_threshold=80, area_threshold=80, margin=10
                )
            except Exception as _fb_e:
                print("find_blobs:", repr(_fb_e))
                invert = not invert
                continue
            if not blobs:
                invert = not invert
                continue

            target_blobs = []
            for b1 in blobs:
                try:
                    h1 = b1.h()
                    if h1 <= 0:
                        continue
                    find_count = 0
                    for b2 in blobs:
                        h2 = b2.h()
                        if h2 <= 0:
                            continue
                        if abs(h1 - h2) < h1 * 0.2 and abs(b1.cy() - b2.cy()) < h1 * 0.3:
                            find_count += 1
                            if find_count > 4:
                                target_blobs.append(b1)
                                break
                except Exception as _blob_e:
                    print("blob row filter:", repr(_blob_e))
                    continue

            if not target_blobs:
                invert = not invert
                continue

            target_blobs.sort(key=lambda b: b.y())

            line_ending = []
            for i in range(len(target_blobs) - 1):
                if abs(target_blobs[i].cy() - target_blobs[i+1].cy()) > target_blobs[i].h() * 0.3:
                    line_ending.append(i)
            line_ending.append(len(target_blobs))

            target_blob_lines = []
            for i, end in enumerate(line_ending):
                start = 0 if i == 0 else line_ending[i-1] + 1
                if end - start > 5:
                    line_blobs = target_blobs[start:end]
                    line_blobs.sort(key=lambda b: b.x())
                    target_blob_lines.append(line_blobs)

            if not target_blob_lines:
                invert = not invert
                continue

            for line in target_blob_lines:
                if len(line) > PLATE_BLOB_KEEP_MAX:
                    del line[0:len(line) - PLATE_BLOB_KEEP_MAX]

            target_blob_max = max(target_blob_lines, key=lambda line: line[0].area() if line else 0)
            target_blob_max = filter_row_blobs_remove_spacing_outliers(target_blob_max)

            if not target_blob_max:
                invert = not invert
                continue

            h_sum = 0
            for b in target_blob_max:
                h_sum += b.h()
            h_avg = round(h_sum / len(target_blob_max))

            x_min = min([b.x() for b in target_blob_max])
            y_min = min([b.y() for b in target_blob_max])
            x_max = max([b.x() + b.w() for b in target_blob_max])
            y_max = max([b.y() + b.h() for b in target_blob_max])

            char_w_est = max(8, round((x_max - x_min) / 6))
            inner_est = max(22, round(h_avg * 1.28))
            province_w = province_slot_width_px(PLATE_MAX_PIXEL_W - 12, inner_est)
            gap_w = max(1, round(char_w_est * 0.15))
            plate_x = max(0, x_min - province_w - gap_w - 2)
            plate_y = max(0, y_min - 3)
            plate_w = min(320 - plate_x, (x_max - x_min) + province_w + gap_w + 6)
            plate_h = min(172 - plate_y, max(round(h_avg * 1.35), y_max - y_min + 6))

            # 测试牌模式下，blob 法常把框左侧估计过宽，整体右移修正
            if TEST_PLATE_MODE and TEST_PLATE_X_SHIFT_RIGHT > 0:
                _dx = min(TEST_PLATE_X_SHIFT_RIGHT, max(0, 320 - (plate_x + plate_w)))
                plate_x += _dx

        # 整牌宽度硬上限，防止偶发噪声导致绿框横向暴涨
        if plate_w > PLATE_MAX_PIXEL_W:
            plate_w = PLATE_MAX_PIXEL_W
            if plate_x + plate_w > 320:
                plate_x = max(0, 320 - plate_w)
        # 绿框补边，减少蓝底被裁掉
        plate_x = max(0, plate_x - PLATE_PAD_X)
        plate_y = max(0, plate_y - PLATE_PAD_Y)
        plate_w = min(320 - plate_x, plate_w + 2 * PLATE_PAD_X)
        plate_h = min(172 - plate_y, plate_h + 2 * PLATE_PAD_Y)
        if plate_w < 40 or plate_h < 16:
            invert = not invert
            continue

        # 绿框时间平滑，抑制单帧突变（模块顶层赋值，勿用 global，MicroPython 会报错）
        if last_plate_rect is not None:
            lx, ly, lw, lh = last_plate_rect
            # 防止单帧估计把绿框突然往左拉很多
            if plate_x < (lx - PLATE_MAX_LEFT_JUMP):
                plate_x = lx - PLATE_MAX_LEFT_JUMP
                if plate_x < 0:
                    plate_x = 0
            s = PLATE_SMOOTH
            plate_x = int((1.0 - s) * lx + s * plate_x)
            plate_y = int((1.0 - s) * ly + s * plate_y)
            plate_w = int((1.0 - s) * lw + s * plate_w)
            plate_h = int((1.0 - s) * lh + s * plate_h)
        last_plate_rect = (plate_x, plate_y, plate_w, plate_h)

        # 蓝牌门控：仅在启用蓝定位时生效；测试牌固定模式下放开该门，避免误拒绝
        if USE_BLUE_LOCATOR and (not TEST_PLATE_MODE):
            if not plate_rect_blue_ok(img, last_plate_rect):
                invert = not invert
                continue

        # ====== 定帧识别门控：绿框稳定 + 清晰度达标 + 每N帧识别一次 ======
        frame_id += 1
        if rect_is_stable(_prev_rect_for_stable, last_plate_rect):
            rect_stable_count += 1
        else:
            rect_stable_count = 1
        _prev_rect_for_stable = last_plate_rect

        do_recognize = (
            (frame_id % max(1, RECOGNIZE_EVERY_N_FRAMES)) == 0
            and rect_stable_count >= PLATE_RECT_STABLE_FRAMES
            and sharp_enough(img_gray2, last_plate_rect)
        )
        if not do_recognize:
            # 仍画框/显示，但不做模板匹配，也不污染投票窗
            img.draw_rectangle((plate_x, plate_y, plate_w, plate_h), color=(0, 255, 0))
            if lcd:
                try:
                    img.draw_string(60, 10, "--", scale=1.2, color=(0, 0, 0))
                    lcd.write(img)
                except Exception:
                    lcd = None
            continue

        # 绘制整块车牌区域（绿色大框）
        img.draw_rectangle((plate_x, plate_y, plate_w, plate_h), color=(0, 255, 0))

        # ---------- 字符分割（优先“示例”风格跳变点分割，失败再回退等宽切格） ----------
        margin_lr = 6
        work_x = plate_x + margin_lr // 2 + PLATE_LEFT_TRIM_PX
        if work_x >= 320:
            work_x = 319
        work_w = plate_w - margin_lr - PLATE_LEFT_TRIM_PX - PLATE_RIGHT_TRIM_PX
        if work_x + work_w > 320:
            work_w = 320 - work_x
        work_rect = (work_x, plate_y, max(1, work_w), plate_h)

        char_boxes = None
        if USE_SAMPLE_STYLE_SEGMENTATION and (not (TEST_PLATE_MODE and TEST_FORCE_FIXED_SEGMENTATION)):
            char_boxes = segment_chars_like_sample(img_gray2, work_rect)

        if char_boxes is None:
            # 回退：原等宽切格
            usable_w = max(42, work_w)
            gap_w = max(1, min(6, usable_w // 40))
            inner_h_box = max(20, plate_h - 4)
            province_w = province_slot_width_px(usable_w, inner_h_box)
            suffix_w = max(36, usable_w - province_w - gap_w)
            cw0 = max(6, suffix_w // 6)
            dot_w = max(PLATE_DOT_GAP_MIN, min(PLATE_DOT_GAP_MAX, int(cw0 * PLATE_DOT_GAP_PER_CHAR)))
            cw = max(6, (suffix_w - dot_w) // 6)
            while cw * 6 + dot_w > suffix_w and dot_w > PLATE_DOT_GAP_MIN:
                dot_w -= 1
                cw = max(6, (suffix_w - dot_w) // 6)

            char_boxes = []
            box_h = max(20, plate_h - 4)
            box_y = plate_y + 2

            p_x = max(0, work_x)
            p_w = max(8, province_w)
            p_w = max(PROVINCE_W_MIN_PX, min(p_w, int(box_h * PROVINCE_ROI_MAX_ASPECT_H) + 2))
            if p_x + p_w > 320:
                p_w = 320 - p_x
            char_boxes.append([p_x, box_y, p_w, box_h])

            start_x = p_x + p_w + gap_w
            if start_x < 0:
                start_x = 0
            lx = start_x
            lw = cw
            if lx + lw > 320:
                lw = max(4, 320 - lx)
            char_boxes.append([lx, box_y, lw, box_h])

            tail0 = start_x + cw + dot_w
            for k in range(5):
                cx = tail0 + k * cw
                clw = cw
                if cx < 0:
                    cx = 0
                if cx + clw > 320:
                    clw = max(4, 320 - cx)
                char_boxes.append([cx, box_y, clw, box_h])
        
        try:
            for i, box in enumerate(char_boxes[:7]):
                img_targets[i].clear()
                
                # box可能是列表或blob
                if hasattr(box, 'x'):
                    x, y, w, h = box.x(), box.y(), box.w(), box.h()
                else:
                    x, y, w, h = box
                
                # 边界检查
                x = max(0, x)
                y = max(0, y)
                w = min(w, 320 - x)
                h = min(h, 172 - y)
                
                if i == 0:
                    scale_x = 40 / h_avg
                    scale_y = 40 / h_avg
                    # 汉字靠格左侧：取样宽度不超过字高，减少右侧蓝底参与匹配
                    w = max(8, min(w, int(h * PROVINCE_ROI_MAX_ASPECT_H) + 2))
                else:
                    scale_x = 40 / h if h > 0 else 1
                    scale_y = 40 / h if h > 0 else 1
                
                roi_x = max(0, x - 2)
                roi_y = max(0, y - 1)
                roi_w = min(w + 5, 320 - roi_x)
                roi_h = min(h + 5, 172 - roi_y)
                
                if roi_w > 0 and roi_h > 0:
                    img_targets[i].draw_image(
                        img_gray2, 0, 0,
                        x_scale=scale_x, y_scale=scale_y,
                        roi=(roi_x, roi_y, roi_w, roi_h)
                    )
                img.draw_rectangle((x, y, w, h), color=(255, 0, 0))
        except Exception as e:
            print("Extract error:", e)
            continue
        
        # 每帧都重置识别结果，避免上一帧残留字符污染当前结果
        license_number = [' '] * 7
        province_hit = False

        # 豫单模板模式：固定首位为“豫”，避免省份位偶发失配导致整帧被 no_province_hit 丢弃
        if PROVINCE_LOAD_MINIMAL_YU_ONLY and len(province_templates) > 0:
            license_number[0] = "豫"
            province_hit = True

        for i in range(len(char_boxes[:7])):
            try:
                if i == 0:
                    if len(province_templates) == 0:
                        print("No province templates!")
                        license_number[i] = '?'
                        continue
                    _hit, _name = match_province_in_target(img_targets[i])
                    if _hit:
                        license_number[i] = _name
                        province_hit = True
                else:
                    h = char_boxes[i][3]
                    w = char_boxes[i][2]
                    # 仅极窄竖条才判为 I，避免整行 cw 偏小时误把多格都打成 I
                    if w > 0 and w < 6 and h >= int(w * 3.8):
                        license_number[i] = 'I'
                        continue
                    
                    if len(alphanumeric_templates) == 0:
                        print("No alphanumeric templates!")
                        license_number[i] = '?'
                        continue

                    def _try_match_char(char, tmpl):
                        try:
                            mr = TEST_MATCHING_RATE if TEST_PLATE_MODE else MATCHING_RATE
                            mr_min = TEST_MATCHING_RATE_MIN if TEST_PLATE_MODE else MATCHING_RATE_MIN
                            _timg = get_template_image(tmpl)  # tmpl 为路径
                            r = img_targets[i].find_template(
                                _timg, mr, step=2, search=image.SEARCH_EX
                            )
                            if (not r) and ((not TEST_PLATE_MODE) or TEST_SECOND_PASS_MATCH):
                                r = img_targets[i].find_template(
                                    _timg, mr_min, step=2, search=image.SEARCH_EX
                                )
                            return r
                        except Exception as _tm_e:
                            print("find_template err:", i, char, repr(_tm_e))
                            return None

                    # 索引 1：发牌机关字母，只跑 A–Z 模板，避免 A 被「4」等数字模板误匹配
                    if i == 1:
                        # 测试牌模式：第二位固定优先 V，命中则直接使用
                        if TEST_PLATE_MODE:
                            for j, tmpl in enumerate(alphanumeric_templates):
                                if j >= len(alphanumeric_names):
                                    continue
                                if alphanumeric_names[j] != "V":
                                    continue
                                if _try_match_char("V", tmpl):
                                    license_number[i] = "V"
                                    break
                            if license_number[i] == "V":
                                continue
                        for j, tmpl in enumerate(alphanumeric_templates):
                            if j >= len(alphanumeric_names):
                                continue
                            char = alphanumeric_names[j]
                            if not ("A" <= char <= "Z"):
                                continue
                            if char in ("I", "O"):
                                continue
                            if _try_match_char(char, tmpl):
                                license_number[i] = char
                                break
                    else:
                        # 测试牌模式：后5位按“3位数字 + 2位字母”限制候选，降低 L/F 噪声
                        if TEST_PLATE_MODE and i >= 2:
                            if i <= 4:
                                _allow = "0123456789"
                            else:
                                _allow = "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
                            for j, tmpl in enumerate(alphanumeric_templates):
                                if j >= len(alphanumeric_names):
                                    continue
                                char = alphanumeric_names[j]
                                if char not in _allow:
                                    continue
                                if char in ("I", "O"):
                                    continue
                                if _try_match_char(char, tmpl):
                                    license_number[i] = char
                                    break
                            continue
                        for j, tmpl in enumerate(alphanumeric_templates):
                            if j >= len(alphanumeric_names):
                                continue
                            char = alphanumeric_names[j]
                            if char in ['I', 'O']:
                                continue
                            if _try_match_char(char, tmpl):
                                license_number[i] = char
                                break
            except Exception as e:
                print("Match error:", e)
                license_number[i] = '?'
                continue

        # 自纠偏（仿示例的“边界修正”思想）：省份可能落在前几格任意一格（边框/左裁边导致）
        # 在前 3 格里找省份模板，命中则把字符整体左移对齐到第 1 格。
        if (not province_hit) and len(province_templates) > 0:
            try:
                hit_at = -1
                hit_name = None
                for idx in (0, 1, 2, 3):
                    if idx >= 7:
                        continue
                    _hit, _name = match_province_in_target(img_targets[idx])
                    if _hit:
                        hit_at = idx
                        hit_name = _name
                        break
                    if hit_at >= 0:
                        break

                if hit_at > 0 and hit_name is not None:
                    # 把省份移到第 0 位，其余字符左移 hit_at 格
                    province_hit = True
                    license_number[0] = hit_name
                    for k in range(1, 7 - hit_at):
                        license_number[k] = license_number[k + hit_at]
                    for k in range(7 - hit_at, 7):
                        license_number[k] = ' '
                    print("Auto align: province found at box", hit_at + 1, "shifted", hit_at)
            except Exception as _ae:
                print("Province auto-align:", repr(_ae))

        # 兜底：若分割框识别不到省份，直接在整牌左侧 ROI 上再做一次省份匹配
        if (not province_hit) and len(province_templates) > 0:
            try:
                px, py, pw, ph = last_plate_rect
                lx = max(0, px + 2)
                ly = max(0, py + 2)
                # 仅“豫”模板模式时，左侧 ROI 取更窄，避免把 V/点混入导致汉字匹配失效
                if PROVINCE_LOAD_MINIMAL_YU_ONLY:
                    lw = min(max(16, int(pw * 0.20)), 320 - lx)
                else:
                    lw = min(max(18, int(pw * 0.30)), 320 - lx)
                lh = min(max(14, ph - 4), 172 - ly)
                if lw > 6 and lh > 6:
                    img_targets[0].draw_image(
                        img_gray2, 0, 0,
                        x_scale=img_targets[0].width() / float(lw),
                        y_scale=img_targets[0].height() / float(lh),
                        roi=(lx, ly, lw, lh)
                    )
                    _hit, _name = match_province_in_target(img_targets[0])
                    if _hit:
                        province_hit = True
                        license_number[0] = _name
                        print("Province fallback hit on left ROI:", _name)
            except Exception as _pf:
                print("Province fallback:", repr(_pf))

        # 测试牌模式：已命中“豫”后，第二位固定为 V，减少二号位抖动
        if TEST_PLATE_MODE and province_hit:
            license_number[1] = 'V'

        plate_raw = ''.join(license_number)
        plate_norm = normalize_plate(plate_raw)
        plate_pos = correct_plate_by_position(plate_norm)

        # 双目标连续测试：根据点后第一位(索引2)的类型快速切换目标，避免长期锁死旧目标
        if TEST_PLATE_MODE and TEST_SWITCH_BY_POS3_HINT and len(TEST_EXPECT_PLATES_ASCII) >= 2:
            hint = plate_raw[2] if len(plate_raw) > 2 else ' '
            want = None
            if '0' <= hint <= '9':
                want = _target_by_pos3_kind(True)
            elif 'A' <= hint <= 'Z':
                want = _target_by_pos3_kind(False)
            elif plate_raw.strip() in ("豫V",):
                # 只有“豫V”且后5位还没出来时，保持用户指定的当前目标，不强行切换
                want = TEST_ACTIVE_TARGET_ASCII if TEST_ACTIVE_TARGET_ASCII in TEST_EXPECT_PLATES_ASCII else ACTIVE_TEST_TARGET
            if (want is not None) and (want != ACTIVE_TEST_TARGET):
                ACTIVE_TEST_TARGET = want
                test_raw_history = []
                plate_history = []
                last_norm_plate = ""
                stable_count = 0
                print("Test target switched by pos3 hint:", ACTIVE_TEST_TARGET)

        # 测试牌模式：跨帧按位累积，解决“单帧经常只识别出 HAV/HA...”的问题
        if TEST_PLATE_MODE and province_hit:
            test_raw_history.append(plate_raw)
            if len(test_raw_history) > TEST_RAW_VOTE_WINDOW:
                test_raw_history.pop(0)
            _test_vote = build_test_plate_from_raw_history(test_raw_history)
            if len(_test_vote) >= 5:
                plate_pos = _test_vote
        elif TEST_PLATE_MODE and (not province_hit):
            # 省份未命中时不清空全部历史，保留近期可用位置信息
            if len(test_raw_history) > TEST_RAW_VOTE_WINDOW:
                test_raw_history = test_raw_history[-TEST_RAW_VOTE_WINDOW:]

        # 测试目标牌为豫系：省份位未命中时，不让该帧污染投票窗口
        if not province_hit:
            plate_history = []
            print("Plate:", plate_raw, "->", plate_norm, "->", plate_pos, "-> vote:", "")
            print("UART skip:", "no_province_hit", "stable=", 0)
            if lcd:
                try:
                    img.draw_string(60, 10, "--", scale=1.2, color=(0, 0, 0))
                    lcd.write(img)
                except Exception as _no_lcd:
                    # 避免反复抛异常导致主循环重启
                    print("LCD disabled (no_province):", repr(_no_lcd))
                    lcd = None
            print("FPS:", clock.fps())
            continue

        # 多帧投票（精度优先）：
        # - 只让“结构合法”的结果进入投票窗口（避免短串/乱码污染）
        # - 任意一帧不合法：直接清窗，宁可慢一点也不让 vote 漂
        if plausible_plate_ascii(plate_pos) and (not _noise_string_for_vote(plate_pos)):
            plate_history.append(plate_pos)
            if len(plate_history) > VOTE_WINDOW:
                plate_history.pop(0)
        else:
            plate_history = []

        plausible_hist = [s for s in plate_history if plausible_plate_ascii(s)]
        if len(plausible_hist) >= 3:
            plate_vote = vote_plate(plausible_hist, prefer_len=7)
        elif len(plausible_hist) >= 1:
            plate_vote = plausible_hist[-1]
        elif len(plate_history) >= 3:
            plate_vote = vote_plate(plate_history, prefer_len=7)
        elif len(plate_history) >= 1:
            plate_vote = plate_history[-1]
        else:
            plate_vote = plate_pos

        # 测试牌模式：当结果与某个目标牌足够接近时，直接收敛到该目标
        if TEST_PLATE_MODE and TEST_FORCE_TARGET_IF_CLOSE:
            best_t = None
            best_s = -1
            tie = False
            for _t in TEST_EXPECT_PLATES_ASCII:
                if len(plate_vote) != len(_t):
                    continue
                _s = same_pos_score(plate_vote, _t)
                if _s > best_s:
                    best_s = _s
                    best_t = _t
                    tie = False
                elif _s == best_s:
                    tie = True
            if (best_t is not None) and (not tie) and (best_s >= TEST_TARGET_MIN_MATCH):
                plate_vote = best_t

        print("Plate:", plate_raw, "->", plate_norm, "->", plate_pos, "-> vote:", plate_vote)

        if len(plate_vote) >= MIN_PLATE_LEN and plausible_plate_ascii(plate_vote):
            if plate_vote == last_norm_plate:
                stable_count += 1
            else:
                last_norm_plate = plate_vote
                stable_count = 1
        else:
            last_norm_plate = ""
            stable_count = 0

        now = millis()
        can_send = (now - last_sent_tick) >= SEND_INTERVAL_MS
        uart_allowed = (not UART_REQUIRE_REAL_PROVINCE) or province_ok_for_uart
        uart_body_ok = plausible_plate_ascii(plate_vote)
        if TEST_PLATE_MODE and TEST_UART_EXACT_ONLY:
            uart_body_ok = uart_body_ok and (plate_vote in TEST_EXPECT_PLATES_ASCII)

        if (
            stable_count >= STABLE_FRAMES_REQUIRED
            and can_send
            and plate_vote != last_sent_plate
            and uart_body_ok
            and uart_allowed
        ):
            uart.write(b"\xFF" + plate_vote.encode("ascii", "ignore") + b"\xFE")
            last_sent_plate = plate_vote
            last_sent_tick = now
            test_dup_streak = 0
            print("UART sent:", plate_vote, "(stable:", stable_count, ")")
        else:
            skip_reason = []
            if stable_count < STABLE_FRAMES_REQUIRED:
                skip_reason.append("stable")
            if not can_send:
                skip_reason.append("cooldown")
            if plate_vote == last_sent_plate:
                skip_reason.append("dup")
            if not uart_body_ok:
                if TEST_PLATE_MODE and TEST_UART_EXACT_ONLY:
                    skip_reason.append("not_target")
                else:
                    skip_reason.append("not_plausible")
            if not uart_allowed:
                skip_reason.append("no_province_tmpl")
            print("UART skip:", ",".join(skip_reason) if skip_reason else "ok", "stable=", stable_count)

            # 双目标模式：若长时间只在 dup，不再等待证据，直接切换目标并清空历史
            if TEST_PLATE_MODE and TEST_UART_EXACT_ONLY and (plate_vote == last_sent_plate):
                test_dup_streak += 1
            else:
                test_dup_streak = 0

            if TEST_PLATE_MODE and len(TEST_EXPECT_PLATES_ASCII) >= 2 and test_dup_streak >= TEST_DUP_FORCE_SWITCH_FRAMES:
                # 仅当当前帧给出了“另一目标类型”的证据时才强制切换，避免单牌测试时误切
                hint = plate_raw[2] if len(plate_raw) > 2 else ' '
                suggest = None
                if '0' <= hint <= '9':
                    suggest = _target_by_pos3_kind(True)
                elif 'A' <= hint <= 'Z':
                    suggest = _target_by_pos3_kind(False)

                if (suggest is not None) and (suggest != ACTIVE_TEST_TARGET):
                    ACTIVE_TEST_TARGET = suggest
                    test_dup_streak = 0
                    test_raw_history = []
                    plate_history = []
                    last_norm_plate = ""
                    stable_count = 0
                    print("Test target force-switched by dup streak:", ACTIVE_TEST_TARGET)
        
        if lcd:
            try:
                _raw = plate_vote if len(plate_vote) > 0 else "--"
                # 屏显仅用可打印 ASCII，避免编码/字间距被误读成「Y K K 1 2」
                _disp = "".join([c for c in _raw if 32 <= ord(c) < 127])
                if len(_disp) == 0:
                    _disp = "--"
                img.draw_string(60, 10, _disp[:16], scale=1.2, color=(0, 0, 0))
                lcd.write(img)
            except Exception as _lcd_e:
                print("LCD skip:", repr(_lcd_e))
        
        print("FPS:", clock.fps())
        
    except Exception as e:
        print("Error:", e)
        import sys
        sys.print_exception(e)
        continue
