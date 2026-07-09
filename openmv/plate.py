import sensor, image, time, display
from pyb import millis, UART
from os import listdir
i = 0
uart = UART(3, 115200)
TEMPLATE_INVERT = False   # 若字模本身已是白字黑底，保持 False；全识别成同一字符时优先检查此项

# Haar 级联辅助定位（命中则缩小后续搜索区域，失败自动回退原流程）
# 该开源 XML 在 OpenMV 上解析不兼容（会触发超大内存申请），默认关闭 Haar 辅助
USE_HAAR_PLATE_ASSIST = False
HAAR_CASCADE_PATHS = [
   "/sdcard/haarcascades/haarcascade_russian_plate_number.xml",
   "/sd/haarcascades/haarcascade_russian_plate_number.xml",
   "/sdcard/haarcascade_plate.xml",
   "/sd/haarcascade_plate.xml",
   "/sdcard/haarcascades/haarcascade_plate.xml",
   "/sd/haarcascades/haarcascade_plate.xml",
   "/sdcard/openmv/haarcascade_plate.xml",
   "/sd/openmv/haarcascade_plate.xml",
]
HAAR_STAGES = 18
HAAR_THRESHOLD = 0.60
HAAR_SCALE = 1.30
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

def _province_label_from_filename(fn):
    stem = fn[:-4]
    if stem.startswith("CN_") and len(stem) >= 5:
        code = stem[3:5]
        return PROVINCE_CODE_TO_HAN.get(code, stem)
    if stem == "province_yu":
        return "豫"
    return stem

def _load_templates_from_paths(paths, map_label=None):
    files = []
    base = None
    for p in paths:
        try:
            files = listdir(p)
            base = p
            break
        except Exception:
            continue
    files.sort()
    names = []
    tmpls = []
    for f in files:
        if not (f.endswith('.pgm') or f.endswith('.png')):
            continue
        try:
            img = image.Image(base + '/' + f)
            if TEMPLATE_INVERT:
                img.invert()
            tmpls.append(img)
            names.append(map_label(f) if map_label else f[:-4])
        except Exception:
            continue
    return names, tmpls

# 省份模板（优先你当前SD卡目录）
library_province, province_template = _load_templates_from_paths(
    ['/sdcard/templates_lp_cn', '/library_province', '/sdcard/library_province'],
    map_label=_province_label_from_filename
)
province_similarity = [0 for _ in range(len(library_province))]
if "豫" in library_province:
   _idx = library_province.index("豫")
   library_province = [library_province[_idx]]
   province_template = [province_template[_idx]]
   province_similarity = [0]

# 字母数字模板（优先你当前SD卡目录）
library_alphanumeric, alphanumeric_template = _load_templates_from_paths(
    ['/sdcard/templates_lp', '/library_alphanumeric', '/sdcard/library_alphanumeric']
)
alphanumeric_similarity = [0 for _ in range(len(library_alphanumeric))]

license_number = []#存储识别到的结果
for n in range(7):
   license_number.append(' ')#省位的h

# 多帧稳定输出参数
VOTE_WINDOW = 8
STABLE_FRAMES_REQUIRED = 4
SEND_INTERVAL_MS = 800
plate_history = []
last_vote = ""
stable_count = 0
last_sent_plate = ""
last_sent_tick = 0

def _match_first(target_img, names, templates, thresholds, allow_fn=None):
   # 改为“唯一命中优先”：避免低阈值下所有字符都命中而固定落到某个字符（如持续J）
   for th in thresholds:
       hits = []
       for idx in range(len(names)):
           ch = names[idx]
           if allow_fn and (not allow_fn(ch)):
               continue
           try:
               blob = target_img.find_template(templates[idx], th, step=2, search=image.SEARCH_EX)
           except Exception:
               blob = None
           if blob:
               hits.append(ch)
       if len(hits) == 1:
           return hits[0]
   return ' '

def _plausible_simple(p):
   # 简单结构校验：省份必须有，第二位必须字母，后位至少有3个非空
   if len(p) != 7:
       return False
   if p[0] == ' ':
       return False
   if not ('A' <= p[1] <= 'Z'):
       return False
   tail_non_blank = 0
   for c in p[2:]:
       if c != ' ':
           tail_non_blank += 1
   return tail_non_blank >= 3

def _plausible_strict(p):
   # 严格门槛：7位都不为空，第二位必须字母（用于串口发送）
   if len(p) != 7:
       return False
   if p[0] == ' ':
       return False
   if not ('A' <= p[1] <= 'Z'):
       return False
   for c in p[2:]:
       if c == ' ':
           return False
   return True

def _vote_plate(cands):
   if not cands:
       return ""
   L = len(cands[0])
   out = []
   for i in range(L):
       cnt = {}
       for s in cands:
           if len(s) <= i:
               continue
           ch = s[i]
           if ch == ' ':
               continue
           cnt[ch] = cnt.get(ch, 0) + 1
       if not cnt:
           out.append(' ')
       else:
           out.append(max(cnt, key=lambda k: cnt[k]))
   return "".join(out)

def _to_uart_ascii_plate(p):
   """
   将识别串转换为 STM32 对接的 ASCII 串：
   - 7位: 豫VH8X96 -> HAVH8X96
   - 去空格，仅保留 [A-Z0-9]
   """
   if p is None:
       return ""
   out = ""
   for ch in p:
       if ch == ' ' or ch == '.':
           continue
       if ch in PROVINCE_HAN_TO_CODE:
           out += PROVINCE_HAN_TO_CODE[ch]
           continue
       if ('a' <= ch <= 'z'):
           ch = chr(ord(ch) - 32)
       if ('A' <= ch <= 'Z') or ('0' <= ch <= '9'):
           out += ch
   return out

def _load_haar_cascade():
   if not USE_HAAR_PLATE_ASSIST:
       return None
   try:
       print("SD haarcascades:", listdir("/sdcard/haarcascades"))
   except Exception:
       pass
   for p in HAAR_CASCADE_PATHS:
       # 方式1：带 stages
       try:
           c = image.HaarCascade(p, stages=HAAR_STAGES)
           print("Haar cascade loaded:", p, "stages=", HAAR_STAGES)
           return c
       except Exception as e1:
           print("Haar load failed(stages):", p, repr(e1))
       # 方式2：不带 stages（部分固件/级联更兼容）
       try:
           c = image.HaarCascade(p)
           print("Haar cascade loaded:", p, "(no stages)")
           return c
       except Exception as e2:
           print("Haar load failed(no stages):", p, repr(e2))
   print("Haar cascade not found, fallback to blob-only.")
   return None

def _detect_plate_roi_by_haar(gray_img, cascade):
   if cascade is None:
       return None
   try:
       feats = gray_img.find_features(cascade, threshold=HAAR_THRESHOLD, scale_factor=HAAR_SCALE)
   except Exception:
       return None
   if not feats:
       return None
   # 取面积最大的候选
   best = None
   best_area = -1
   for r in feats:
       try:
           x, y, w, h = r
       except Exception:
           continue
       a = w * h
       if a > best_area:
           best_area = a
           best = (x, y, w, h)
   return best
sensor.reset() # Initialize the camera sensor.
# sensor.set_framesize(sensor.QQVGA2)  # 128x160大小的特定液晶屏。
sensor.set_pixformat(sensor.RGB565) # or sensor.RGB565
sensor.set_framesize(sensor.QVGA) # or sensor.QVGA (or others)
sensor.set_windowing(320,172)
sensor.set_contrast(2)
clock = time.clock() # Tracks FPS.

img_GRAYSCALE = sensor.alloc_extra_fb(320,172,sensor.GRAYSCALE)
img_GRAYSCALE_2 = sensor.alloc_extra_fb(320,172,sensor.GRAYSCALE)
img_targets = []
img_targets.append(sensor.alloc_extra_fb(35,55,sensor.GRAYSCALE))
for n in range(6):
   img_targets.append(sensor.alloc_extra_fb(28,45,sensor.GRAYSCALE))
haar_cascade = _load_haar_cascade()
haar_enabled = haar_cascade is not None

invert = False#是否反转颜色，以适应黑色字符的车牌
lcd = display.SPIDisplay()
while(True):
    clock.tick() # Track elapsed milliseconds between snapshots().
    target_blob_max=None
    img = sensor.snapshot() # Take a picture and return the image.

    #一、处理图像并选中数字字母区域
    img_GRAYSCALE.draw_image(img,0,0) #原图绘制到灰度画布上，用于定位字符
    if invert:
        img_GRAYSCALE.invert()
    img_GRAYSCALE_2.draw_image(img_GRAYSCALE,0,0) #复制第二份灰度图，用于识别
    # img_GRAYSCALE_2.binary([(10,255)]) #按阈值二值化
    img_GRAYSCALE.laplacian(3)  #通过拉普拉斯变换，突出色彩分界线（数值越大效果越好，但越慢。所以用最小值，再提高画面亮度）
    img_GRAYSCALE.gamma_corr(gamma=1.2,contrast=25) #提高画面伽马值、对比度、亮度
    #识别浅色区域，加上尺寸、连续度的限制
    haar_roi = _detect_plate_roi_by_haar(img_GRAYSCALE, haar_cascade) if haar_enabled else None
    if haar_roi:
       hx, hy, hw, hh = haar_roi
       # 略扩张，避免只截到牌体内部
       hx = max(0, hx - 8)
       hy = max(0, hy - 4)
       hw = min(320 - hx, hw + 16)
       hh = min(172 - hy, hh + 8)
       img.draw_rectangle((hx, hy, hw, hh), color=(0, 255, 255))
       blobs = img_GRAYSCALE.find_blobs([(2,255)], roi=(hx, hy, hw, hh), x_stride=4,y_stride=2, pixels_threshold=60, area_threshold=60, margin=8)
    else:
       blobs = img_GRAYSCALE.find_blobs([(2,255)], x_stride=4,y_stride=2,pixels_threshold=80, area_threshold=80, margin=10)
    if not blobs:
       invert = not invert
       lcd.write(img)
       continue

    #二、通过设置条件判断，筛选出符合车牌号特征的区域集，并排序，待模板匹配使用
    timer = millis()#用于计这段消耗的时间，如果耗时过长，需要优化或移植到底层（C语言）
    #1.遍历筛选所有识别结果,筛选条件：自己和其他4个以上元素 高度 和 y坐标 相互相似的目标
    target_blobs=[]
    for n1 in range(len(blobs)):
       find_out_times = 0
       for n2 in range(len(blobs)):
           #判断高度差、Y轴差异度
           if abs(blobs[n1].h() - blobs[n2].h()) < (blobs[n1].h() * 0.2) and \
           abs(blobs[n1].cy() - blobs[n2].cy()) < (blobs[n1].h() * 0.3):
               find_out_times += 1
               if find_out_times > 4:#超过5次符合，记录
                   target_blobs.append(blobs[n1])
                   break
    #2.结果按y轴排序
    target_blobs.sort(key = lambda b: b.y())#按选择框cy排序

    #3.在结果中记录每行的结束序号，比如 [(45,3),(85,3),(78,20),(23,20)],第二个元素为第一行的结束序号
    line_ending = []#记录每一行目标最后一项的序号
    for n in range(0,len(target_blobs)-1):
       if  abs(target_blobs[n].cy() - target_blobs[n+1].cy()) > target_blobs[n].h()*0.3:#两个Y差大于40%
           line_ending.append(n)
    line_ending.append(len(target_blobs))#上述方法不能记录最后一行，在此加入最后一行结束点。如果target_blobs没有内容，会加入0

    #4.分割每行，并排序、淘汰每行少于5个的元素
    target_blob_lines = []#存储以行为单位，完成排序的目标坐标结果
    if line_ending and line_ending != [0]:#如果有目标
       for n in range(len(line_ending)):#循环遍历
           if n == 0:#首行
               if line_ending[n] - 0 > 5:    #如果大于5个元素，存储内容，否则抛弃内容
                   target_blob_lines.append(target_blobs[ : line_ending[n]])#转存内容
                   target_blob_lines[-1].sort(key = lambda b: b[0])#按选择框x坐标排序
           elif line_ending[n] - line_ending[n-1] > 5: #非首行。如果大于5个元素，存储内容，否则抛弃内容
               target_blob_lines.append(target_blobs[line_ending[n-1] + 1 : line_ending[n]])#转存内容
               target_blob_lines[-1].sort(key = lambda b: b[0])#按选择框x坐标排序
    #5.进一步筛选掉每行，前后出现的，间距不符的元素
    n = 0
    while True:
       for n in range(len(target_blob_lines)):#行遍历，如果最后一位与倒数第二位间距，不符合倒数第二、第三位间距，则删除最后一位
           if  (target_blob_lines[n][-2].cx() - target_blob_lines[n][-3].cx())*0.8 <\
               (target_blob_lines[n][-1].cx() - target_blob_lines[n][-2].cx()) > \
               (target_blob_lines[n][-2].cx() - target_blob_lines[n][-3].cx())*1.2:
               del target_blob_lines[n][-1]
               break
       if n >= len(target_blob_lines)-1:
           break
    for n in range(len(target_blob_lines)):#行遍历，删除 从后向前数6个以外的其他元素
       if len(target_blob_lines[n]) > 6:
           del target_blob_lines[n][0 : len(target_blob_lines[n]) - 6]

    #6.只保留像素最大的一行
    if target_blob_lines:#如果有目标
       target_blob_max = max(target_blob_lines, key = lambda line: sum([b.area() for b in line]))#保留整行像素总和最大的行

       #7.补充省位选择框
       h_average = 0   #平均高度
       spacing_average = 0 #平均间距(后段相邻部分)
       y_difference_average = 0 #平均y坐标差(后段相邻部分)
       length = len(target_blob_max)
       for n in range(length):
           h_average += target_blob_max[n].h()
           if n > 1:
               spacing_average += target_blob_max[n].cx() - target_blob_max[n-1].cx()
               y_difference_average += target_blob_max[n].cy() - target_blob_max[n-1].cy()
       if length < 6:
           target_blob_max = None
       else:
           h_average = round(h_average / length)
           w_average = round(h_average / 2)
           if (length - 2) > 0:
               spacing_average = round(spacing_average / (length - 2 ))
               y_difference_average = round(y_difference_average / (length - 2))
           else:
               spacing_average = round(w_average * 1.4)
               y_difference_average = 0

           target_blob_max.insert(0, [round(target_blob_max[0].x() - (spacing_average * 1.2)),\
                                      round(target_blob_max[0].y() - (y_difference_average * 1.1)-2),\
                                      round(w_average * 1.4),\
                                      round(h_average * 1.2)])
    else:   #没找到目标
       invert = not invert #通知反转画面，以待识别黑色字符
    if target_blob_max:
       try:    #此处常遇报错，用try跳过报错
           for n in range(min(len(target_blob_max), len(img_targets))):
               img_targets[n].clear()
               if n == 0:
                   img_targets[n].draw_image(img_GRAYSCALE_2, 0, 0, x_scale = 40 / h_average, y_scale = 40 / h_average,\
                                          roi = (target_blob_max[n][0] - 2,\
                                                 target_blob_max[n][1] - 1,\
                                                 target_blob_max[n][2] + 5,\
                                                 target_blob_max[n][3] + 5))
               else:
                   img_targets[n].draw_image(img_GRAYSCALE_2, 0, 0, x_scale = 40 / target_blob_max[n][3], y_scale = 40 / target_blob_max[n][3],\
                                          roi = (target_blob_max[n][0] - 1,\
                                                 target_blob_max[n][1] - 1,\
                                                 target_blob_max[n][2] + 5,\
                                                 target_blob_max[n][3] + 5))

               img.draw_rectangle(target_blob_max[n][:4], color=(255,0,0))
               # img.draw_image(img_targets[n], n*30, 0,)   #将剪切结果绘制到主画布上，以观察效果
       except Exception:
           continue
       # 每帧先清空识别结果，避免残留
       for k in range(7):
           license_number[k] = ' '

       for n in range(min(len(target_blob_max), len(img_targets))):
           try:
               if n == 0:
                   # 省份位：阈值递降，首个有效命中
                   license_number[n] = _match_first(
                       img_targets[n],
                       library_province,
                       province_template,
                       [0.74, 0.64, 0.54, 0.44]
                   )
               else:
                   # 第2位（发牌机关）只允许字母；其余位允许数字字母，排除 I/O
                   if n == 1:
                       license_number[n] = _match_first(
                           img_targets[n],
                           library_alphanumeric,
                           alphanumeric_template,
                           [0.74, 0.64, 0.54, 0.44],
                           allow_fn=lambda c: ('A' <= c <= 'Z') and (c != 'I') and (c != 'O')
                       )
                   else:
                       license_number[n] = _match_first(
                           img_targets[n],
                           library_alphanumeric,
                           alphanumeric_template,
                           [0.74, 0.64, 0.54, 0.44],
                           allow_fn=lambda c: c != 'I' and c != 'O'
                       )
           except BaseException:
               license_number[n] = ' '
       plate_s = license_number[0] + license_number[1] + license_number[2] + license_number[3] + license_number[4] + license_number[5] + license_number[6]
       # 多帧入窗：仅把“基础结构可用”的结果纳入投票
       if _plausible_simple(plate_s):
           plate_history.append(plate_s)
           if len(plate_history) > VOTE_WINDOW:
               plate_history.pop(0)
       elif len(plate_history) > 0:
           # 当前帧无效时，不立即清空窗口，保留短时稳定性
           pass

       plate_vote = _vote_plate(plate_history) if plate_history else ""
       if _plausible_simple(plate_vote):
           if plate_vote == last_vote:
               stable_count += 1
           else:
               last_vote = plate_vote
               stable_count = 1
       else:
           last_vote = ""
           stable_count = 0

       now = millis()
       can_send = (now - last_sent_tick) >= SEND_INTERVAL_MS
       if _plausible_strict(plate_vote) and stable_count >= STABLE_FRAMES_REQUIRED and can_send and plate_vote != last_sent_plate:
           uart_payload = _to_uart_ascii_plate(plate_vote)
           if len(uart_payload) >= 8:
               # STM32 帧格式: 0xFF + ASCII内容 + 0xFE
               uart.write(b"\xFF" + uart_payload.encode("ascii", "ignore") + b"\xFE")
           last_sent_plate = plate_vote
           last_sent_tick = now
           stable_count = 0
           print("稳定输出 省份："+plate_vote[0] +" 车牌号："+ plate_vote[1:] + " UART:" + uart_payload)
       else:
           print("候选:", plate_s, " vote:", plate_vote, " stable=", stable_count)
       img.draw_image(img_targets[0], 10, 0, )
       a = license_number[1]  + license_number[2] +license_number[3] + license_number[4] + license_number[5] + license_number[6]
       img.draw_string(60, 10, a, scale=1.4, color=(0, 0, 0))
    lcd.write(img)


