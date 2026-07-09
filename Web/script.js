// 停车场配置
const PARKING_CONFIG = {
    rows: 11,
    cols: 11,
    entrance: { x: 0, y: 0 },
    exit: { x: 6, y: 10 },
    isRoad: function(i, j) {
        if (i % 2 === 1) return true;
        if (j === 0 || j === 5 || j === 10) return true;
        return false;
    }
};

// 模拟数据
let parkingData = [];
const SENSOR_SPOTS = [
    { x: 2, y: 2 }, // parking1
    { x: 4, y: 4 }  // parking2
];

function isSensorSpot(x, y) {
    return SENSOR_SPOTS.some(spot => spot.x === x && spot.y === y);
}

// 初始化停车场数据
function initParkingData() {
    parkingData = [];
    for (let i = 0; i < PARKING_CONFIG.rows; i++) {
        let row = [];
        for (let j = 0; j < PARKING_CONFIG.cols; j++) {
            if (PARKING_CONFIG.isRoad(i, j)) {
                row.push({ type: 'road', status: 'road' });
            } else {
                // 除超声波检测的两个车位外，其余车位默认视为已占用
                row.push({ type: 'parking', status: isSensorSpot(i, j) ? 'empty' : 'occupied' });
            }
        }
        parkingData.push(row);
    }
    
    // 设置入口和出口
    parkingData[PARKING_CONFIG.entrance.x][PARKING_CONFIG.entrance.y] = 
        { type: 'entrance', status: 'entrance' };
    parkingData[PARKING_CONFIG.exit.x][PARKING_CONFIG.exit.y] = 
        { type: 'exit', status: 'exit' };
}

// 生成停车场网格
function generateParkingGrid() {
    const grid = document.getElementById('parkingGrid');
    grid.innerHTML = '';
    
    for (let i = 0; i < PARKING_CONFIG.rows; i++) {
        for (let j = 0; j < PARKING_CONFIG.cols; j++) {
            const cell = document.createElement('div');
            cell.className = `parking-cell ${parkingData[i][j].status}`;
            cell.dataset.x = i;
            cell.dataset.y = j;
            
            // 添加坐标标签
            cell.title = `[${i}][${j}]`;
            
            grid.appendChild(cell);
        }
    }
}

// 更新停车场显示
function updateParkingDisplay() {
    const cells = document.querySelectorAll('.parking-cell');
    cells.forEach(cell => {
        const x = parseInt(cell.dataset.x);
        const y = parseInt(cell.dataset.y);
        
        cell.className = `parking-cell ${parkingData[x][y].status}`;
    });
    
    updateStats();
}

// 更新统计信息
function updateStats() {
    let totalParking = 0;
    let occupied = 0;
    let empty = 0;
    
    for (let i = 0; i < PARKING_CONFIG.rows; i++) {
        for (let j = 0; j < PARKING_CONFIG.cols; j++) {
            if (parkingData[i][j].type === 'parking') {
                totalParking++;
                if (parkingData[i][j].status === 'occupied') {
                    occupied++;
                } else {
                    empty++;
                }
            }
        }
    }
    
    const usageRate = totalParking > 0 ? ((occupied / totalParking) * 100).toFixed(1) : 0;
    
    document.getElementById('totalSpots').textContent = totalParking;
    document.getElementById('emptySpots').textContent = empty;
    document.getElementById('occupiedSpots').textContent = occupied;
    document.getElementById('usageRate').textContent = usageRate + '%';
    
    // 更新最后更新时间
    const now = new Date();
    document.getElementById('updateTime').textContent = 
        `最后更新: ${now.toLocaleTimeString()}`;
}

// 添加日志
function addLog(message, type = 'info') {
    const logContent = document.getElementById('logContent');
    const logItem = document.createElement('div');
    logItem.className = `log-item ${type}`;
    
    const now = new Date();
    const time = now.toLocaleTimeString();
    logItem.textContent = `[${time}] ${message}`;
    
    logContent.insertBefore(logItem, logContent.firstChild);
    
    // 限制日志数量
    while (logContent.children.length > 20) {
        logContent.removeChild(logContent.lastChild);
    }
}

// 模拟数据变化
function simulateData() {
    // 随机选择几个停车位改变状态
    const parkingSpots = [];
    for (let i = 0; i < PARKING_CONFIG.rows; i++) {
        for (let j = 0; j < PARKING_CONFIG.cols; j++) {
            if (parkingData[i][j].type === 'parking') {
                parkingSpots.push({ x: i, y: j });
            }
        }
    }
    
    // 随机改变3-5个停车位的状态
    const changeCount = Math.floor(Math.random() * 3) + 3;
    for (let k = 0; k < changeCount; k++) {
        const randomIndex = Math.floor(Math.random() * parkingSpots.length);
        const spot = parkingSpots[randomIndex];
        
        const newStatus = Math.random() > 0.5 ? 'occupied' : 'empty';
        const oldStatus = parkingData[spot.x][spot.y].status;
        
        if (newStatus !== oldStatus) {
            parkingData[spot.x][spot.y].status = newStatus;
            addLog(`车位 [${spot.x}][${spot.y}] 状态变化: ${oldStatus} -> ${newStatus}`, 
                   newStatus === 'occupied' ? 'warning' : 'success');
        }
    }
    
    updateParkingDisplay();
    addLog('数据已更新（模拟）', 'info');
}

// MQTT连接配置（阿里云IoT）
const MQTT_CONFIG = {
    // 阿里云IoT设备信息
    productKey: 'k1uoeMNdaXi',
    deviceName: 'Web001',
    // 连接方式:
    // static: 使用控制台复制出来的 clientId/username/passwd（你当前提供的参数）
    // dynamic: 使用 deviceSecret 在浏览器计算签名
    authMode: 'static',
    deviceSecret: '',
    staticAuth: {
        clientId: 'k1uoeMNdaXi.Web001|securemode=2,signmethod=hmacsha256,timestamp=1776852555335|',
        username: 'Web001&k1uoeMNdaXi',
        password: 'd93ff14f987e421d0270a79abe82c7cc7ba1b27f0dd705c46d7de20f5b291d48'
    },
    
    // MQTT服务器地址
    host: 'iot-06z00i4rmqe2hzc.mqtt.iothub.aliyuncs.com',
    // 浏览器端走 WSS（443）+ /mqtt
    wsPort: 443,
    wsPath: '/mqtt',

    // 与规则引擎脚本 writeIotTopic(1004, "...") 保持一致
    topic: '/k1uoeMNdaXi/Web001/user/parking/status',
    // 主动同步时，请求停车设备上报最新属性
    parkingDeviceName: 'parking'
};

let mqttClient = null;

function strToUint8(str) {
    return new TextEncoder().encode(str);
}

async function hmacSha256Hex(keyStr, msgStr) {
    const key = await crypto.subtle.importKey(
        'raw',
        strToUint8(keyStr),
        { name: 'HMAC', hash: 'SHA-256' },
        false,
        ['sign']
    );
    const sig = await crypto.subtle.sign('HMAC', key, strToUint8(msgStr));
    const bytes = new Uint8Array(sig);
    return Array.from(bytes).map(b => b.toString(16).padStart(2, '0')).join('');
}

async function buildAliyunMqttAuth() {
    const pk = MQTT_CONFIG.productKey;
    const dn = MQTT_CONFIG.deviceName;
    const ds = MQTT_CONFIG.deviceSecret;
    const ts = Date.now().toString();

    // 与硬件侧一致的 clientId 形态
    const clientId = `${pk}.${dn}|securemode=2,signmethod=hmacsha256,timestamp=${ts}|`;
    const username = `${dn}&${pk}`;

    // Aliyun MQTT 签名串（常用格式）
    // password = HMAC_SHA256(deviceSecret, "clientId${clientId}deviceName${dn}productKey${pk}timestamp${ts}")
    const signContent = `clientId${clientId}deviceName${dn}productKey${pk}timestamp${ts}`;
    const password = await hmacSha256Hex(ds, signContent);

    return { clientId, username, password };
}

// MQTT连接（需要paho-mqtt库）
async function connectMQTT() {
    addLog('尝试连接阿里云IoT平台...', 'info');
    
    try {
        let auth = null;
        if (MQTT_CONFIG.authMode === 'static') {
            auth = MQTT_CONFIG.staticAuth;
            if (!auth.clientId || !auth.username || !auth.password) {
                addLog('静态认证参数缺失，请检查 staticAuth 配置', 'error');
                updateConnectionStatus(false);
                return;
            }
        } else {
            if (!MQTT_CONFIG.deviceSecret) {
                addLog('动态签名模式需填写 DeviceSecret', 'error');
                updateConnectionStatus(false);
                return;
            }
            if (!window.crypto || !crypto.subtle) {
                addLog('当前浏览器不支持 WebCrypto（crypto.subtle），无法计算 HMAC-SHA256 签名', 'error');
                updateConnectionStatus(false);
                return;
            }
            auth = await buildAliyunMqttAuth();
        }

        // 使用 URL 形式创建 Paho 客户端（最稳定）
        const wsUrl = `wss://${MQTT_CONFIG.host}:${MQTT_CONFIG.wsPort}${MQTT_CONFIG.wsPath}`;
        mqttClient = new Paho.MQTT.Client(wsUrl, auth.clientId);
        
        mqttClient.onConnectionLost = onConnectionLost;
        mqttClient.onMessageArrived = onMessageArrived;
        
        const connectOptions = {
            onSuccess: onConnect,
            onFailure: onFail,
            useSSL: true,
            userName: auth.username,
            password: auth.password,
            keepAliveInterval: 60,
            cleanSession: true
        };
        
        mqttClient.connect(connectOptions);
    } catch (error) {
        addLog('MQTT连接错误: ' + error.message, 'error');
        updateConnectionStatus(false);
    }
}

function onConnect() {
    addLog('MQTT连接成功！', 'success');
    updateConnectionStatus(true);
    
    // 订阅主题
    mqttClient.subscribe(MQTT_CONFIG.topic);
    addLog('已订阅主题: ' + MQTT_CONFIG.topic, 'info');
}

function onFail(responseObject) {
    addLog('MQTT连接失败: ' + responseObject.errorMessage, 'error');
    updateConnectionStatus(false);
}

function onConnectionLost(responseObject) {
    if (responseObject.errorCode !== 0) {
        addLog('MQTT连接断开: ' + responseObject.errorMessage, 'error');
        updateConnectionStatus(false);
    }
}

function onMessageArrived(message) {
    addLog('收到消息: ' + message.payloadString, 'info');
    
    try {
        const data = JSON.parse(message.payloadString);
        handleParkingData(data);
    } catch (error) {
        addLog('消息解析错误: ' + error.message, 'error');
    }
}

function requestCloudSync() {
    if (!mqttClient || !mqttClient.isConnected || !mqttClient.isConnected()) {
        addLog('未连接云平台，无法发起主动同步', 'warning');
        return;
    }

    const reqTopic = `/sys/${MQTT_CONFIG.productKey}/${MQTT_CONFIG.parkingDeviceName}/thing/service/property/get`;
    const reqPayload = JSON.stringify({
        id: `web_sync_${Date.now()}`,
        version: '1.0',
        method: 'thing.service.property.get',
        params: ['parking1', 'parking2', 'license_plate1', 'license_plate2', 'license_plate', 'fee']
    });

    const msg = new Paho.MQTT.Message(reqPayload);
    msg.destinationName = reqTopic;
    msg.qos = 0;

    try {
        mqttClient.send(msg);
        addLog('已发起主动同步请求，等待云端返回最新属性...', 'info');
    } catch (error) {
        addLog('主动同步请求发送失败: ' + error.message, 'error');
    }
}

function pickValue(data, key) {
    if (!data) return undefined;

    // 1) 你的固件直传/规则转发常见结构：{ params: { key: value } }
    if (data.params && data.params[key] !== undefined) {
        return data.params[key];
    }
    // 2) 有些云侧消息是扁平结构：{ key: value }
    if (data[key] !== undefined) {
        return data[key];
    }
    // 3) 云产品流转常见结构：{ items: { key: { value: ... } } }
    if (data.items && data.items[key] && data.items[key].value !== undefined) {
        return data.items[key].value;
    }
    return undefined;
}

function handleParkingData(data) {
    const parking1 = pickValue(data, 'parking1');
    const parking2 = pickValue(data, 'parking2');
    const plate1 = pickValue(data, 'license_plate1');
    const plate2 = pickValue(data, 'license_plate2');
    const checkoutPlate = pickValue(data, 'license_plate');
    const fee = pickValue(data, 'fee');

    if (parking1 !== undefined) {
        parkingData[2][2].status = Number(parking1) ? 'occupied' : 'empty';
        addLog(`车位1状态更新: ${Number(parking1) ? '已占用' : '空余'}`, 'success');
    }

    if (parking2 !== undefined) {
        parkingData[4][4].status = Number(parking2) ? 'occupied' : 'empty';
        addLog(`车位2状态更新: ${Number(parking2) ? '已占用' : '空余'}`, 'success');
    }

    if (plate1 !== undefined) {
        document.getElementById('plate1').textContent = plate1 || '--';
        addLog(`license_plate1: ${plate1}`, 'info');
    }

    if (plate2 !== undefined) {
        document.getElementById('plate2').textContent = plate2 || '--';
        addLog(`license_plate2: ${plate2}`, 'info');
    }

    if (checkoutPlate !== undefined) {
        addLog(`出库车牌: ${checkoutPlate}`, 'warning');
    }

    if (fee !== undefined) {
        document.getElementById('checkoutFee').textContent = `${fee}`;
        addLog(`费用: ${fee} 元`, 'warning');
    }

    updateParkingDisplay();
}

function updateConnectionStatus(connected) {
    const statusElement = document.getElementById('connectionStatus');
    if (connected) {
        statusElement.textContent = '● 已连接';
        statusElement.className = 'status connected';
    } else {
        statusElement.textContent = '● 未连接';
        statusElement.className = 'status disconnected';
    }
}

// 生成简单的MQTT密码（实际需要根据设备密钥生成）
function getMqttPassword() {
    return '7b74885316f0ed29ae4fd8612edb466e91fc93ba3b6d2253aee10b0419e21a20';
}

// 刷新数据
function refreshData() {
    addLog('手动刷新：主动同步云平台数据', 'info');

    if (mqttClient && mqttClient.isConnected && mqttClient.isConnected()) {
        requestCloudSync();
        return;
    }

    addLog('当前未连接云平台，正在尝试重连...', 'warning');
    connectMQTT().then(() => {
        // 给连接建立一点时间，再发同步请求
        setTimeout(requestCloudSync, 800);
    });
}

// 页面加载完成后初始化
document.addEventListener('DOMContentLoaded', function() {
    initParkingData();
    generateParkingGrid();
    updateStats();
    addLog('停车场监控系统初始化完成', 'info');
    
    // 绑定按钮事件
    document.getElementById('refreshBtn').addEventListener('click', refreshData);
    document.getElementById('simulateBtn').addEventListener('click', simulateData);
    
    // 尝试连接MQTT（如果库已加载）
    if (typeof Paho !== 'undefined') {
        connectMQTT();
    } else {
        addLog('MQTT库未加载，使用模拟数据模式', 'warning');
    }
    
    // 自动模拟刷新（已禁用）：
    // 之前用于离线演示，现改为仅依赖云端真实消息驱动页面更新。
    // setInterval(function() {
    //     if (document.visibilityState !== 'visible') return;
    //     if (mqttClient && mqttClient.isConnected && mqttClient.isConnected()) return;
    //     simulateData();
    // }, 5000);
});

// 页面可见性变化时处理
document.addEventListener('visibilitychange', function() {
    if (document.visibilityState === 'visible') {
        addLog('页面恢复可见，更新数据...', 'info');
        updateParkingDisplay();
    }
});
