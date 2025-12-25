#include "mainwindow.h"

#include <cstdint>
#include <cstddef>
#include <cstring>
#include <memory>
#include <algorithm>
#include <cmath>

//Qt related header files (for file/directory operations)
#include <QDir>
#include <QFile>

//===== Your BUFSZ definition (remain unchanged) =====
#define BUFSZ 16590848 // Original approx. 16 MB

//===== Shared memory fixed offset constant (consistent with the write end) =====
static constexpr size_t kHeaderOff  = 1024;  //Your original image header: the storage starting point of X/Y/bitDepth
static constexpr size_t kFlagOff    = 2047;  //Frame status flag
static constexpr size_t kPayloadOff = 2048;  //Starting point of pixel data

// ===== Added V2 header (placed at 0..1023, does not affect the old logic) =====
#pragma pack(push,1)
struct ShmHdrV2 {
    uint32_t magic;       // 'PHD2' = 0x32444850
    uint16_t version;     // 0x0002
    uint16_t coding;      // 0=RAW, 1=RLE, 2=NEAREST
    uint32_t payloadSize; // Number of bytes actually written to the pixel area (for safe memcpy/decoding)

    uint32_t origW;       // Original image width
    uint32_t origH;       // Original image height
    uint32_t outW;        //Actual writing frame width (RAW/RLE is the same as orig; NEAREST is deflated)
    uint32_t outH;        //actual writing frame height
    uint16_t bitDepth;    // 8 or 16
    uint16_t scale;       // s for NEAREST; RAW/RLE=1
    uint32_t reserved[8]; // reserved
};
#pragma pack(pop)

static constexpr uint32_t SHM_MAGIC = 0x32444850; // 'PHD2'
static constexpr uint16_t SHM_VER   = 0x0002;
enum : uint16_t { CODING_RAW=0, CODING_RLE=1, CODING_NEAREST=2 };

INDI::BaseDevice *dpMount = nullptr, *dpGuider = nullptr, *dpPoleScope = nullptr;
INDI::BaseDevice *dpMainCamera = nullptr, *dpFocuser = nullptr, *dpCFW = nullptr;

DriversList drivers_list;
std::vector<DevGroup> dev_groups;
std::vector<Device> devices;

DriversListNew drivers_list_new;

SystemDevice systemdevice;
SystemDeviceList systemdevicelist;

// QUrl websocketUrl(QStringLiteral("ws://192.168.2.31:8600"));
QUrl websocketUrl;

// Define static member variables instance
MainWindow *MainWindow::instance = nullptr;

std::string MainWindow::getBuildDate()
{ // compile time date
    static const std::map<std::string, std::string> monthMap = {
        {"Jan", "01"}, {"Feb", "02"}, {"Mar", "03"}, {"Apr", "04"}, {"May", "05"}, {"Jun", "06"}, {"Jul", "07"}, {"Aug", "08"}, {"Sep", "09"}, {"Oct", "10"}, {"Nov", "11"}, {"Dec", "12"}};

    std::string date = __DATE__;
    std::stringstream dateStream(date);
    std::string month, day, year;
    dateStream >> month >> day >> year;

    return year + monthMap.at(month) + (day.size() == 1 ? "0" + day : day);
}

MainWindow::MainWindow(QObject *parent) : QObject(parent)
{
    // Initialize the polar calibration object to nullptr
    polarAlignment = nullptr;

    // Initialize camera parameters
    glFocalLength = 0;
    glCameraSize_width = 0.0;
    glCameraSize_height = 0.0;

    system_timer = new QTimer(this); // For system monitoring
    connect(system_timer, &QTimer::timeout, this, &MainWindow::updateCPUInfo);
    system_timer->start(3000);

    Logger::Initialize();
    getHostAddress();

    wsThread = new WebSocketThread(websockethttpUrl, websockethttpsUrl, this);

    connect(wsThread, &WebSocketThread::receivedMessage, this, &MainWindow::onMessageReceived);
    wsThread->start();
    Logger::wsThread = wsThread;

    // Remember the current instance
    instance = this;

    // Install a custom message handler
    // qInstallMessageHandler(customMessageHandler);

    InitPHD2();

    initINDIServer();
    initINDIClient();

    initGPIO();

    readDriversListFromFiles("/usr/share/indi/drivers.xml", drivers_list, dev_groups, devices);

    Tools::InitSystemDeviceList();
    Tools::initSystemDeviceList(systemdevicelist);
    Tools::makeConfigFile();
    // Cancel: No longer read/save the Zhongtian flip persistence state (unify the geometric method needsFlip)
    Tools::makeImageFolder();
    connect(Tools::getInstance(), &Tools::parseInfoEmitted, this, &MainWindow::onParseInfoEmitted);

    m_thread = new QThread;
    m_threadTimer = new QTimer;
    m_threadTimer->setInterval(200);
    m_threadTimer->moveToThread(m_thread);
    connect(m_threadTimer, &QTimer::timeout, this, &MainWindow::onTimeout);
    connect(m_thread, &QThread::finished, m_threadTimer, &QTimer::stop);
    connect(m_thread, &QThread::destroyed, m_threadTimer, &QTimer::deleteLater);
    connect(m_thread, &QThread::started, m_threadTimer, QOverload<>::of(&QTimer::start));
    m_thread->start();

    PHDControlGuide_thread = new QThread;
    PHDControlGuide_threadTimer = new QTimer;
    PHDControlGuide_threadTimer->setInterval(200);
    PHDControlGuide_threadTimer->moveToThread(PHDControlGuide_thread);
    connect(PHDControlGuide_threadTimer, &QTimer::timeout, this, &MainWindow::onPHDControlGuideTimeout);
    connect(PHDControlGuide_thread, &QThread::finished, PHDControlGuide_threadTimer, &QTimer::stop);
    connect(PHDControlGuide_thread, &QThread::destroyed, PHDControlGuide_threadTimer, &QTimer::deleteLater);
    connect(PHDControlGuide_thread, &QThread::started, PHDControlGuide_threadTimer, QOverload<>::of(&QTimer::start)); // New
    PHDControlGuide_thread->start();
    // getConnectedSerialPorts();

    // ESC control initialization
    focusMoveTimer = new QTimer(this);
    connect(focusMoveTimer, &QTimer::timeout, this, &MainWindow::HandleFocuserMovementDataPeriodically);

    emit wsThread->sendMessageToClient("ServerInitSuccess");

}

MainWindow::~MainWindow()
{
    // Clean up polar alignment objects
    if (polarAlignment != nullptr)
    {
        polarAlignment->stopPolarAlignment();
        delete polarAlignment;
        polarAlignment = nullptr;
    }

    system("pkill indiserver");
    system("rm -f /tmp/myFIFO");

    wsThread->quit();
    wsThread->wait();
    delete wsThread;

    // Clean up static instances
    instance = nullptr;
}

void MainWindow::getHostAddress()
{
    int retryCount = 0;
    const int maxRetries = 20;
    const int waitTime = 5000; // 5 seconds

    while (retryCount < maxRetries)
    {
        QList<QNetworkInterface> interfaces = QNetworkInterface::allInterfaces();
        bool found = false;

        foreach (const QNetworkInterface &interface, interfaces)
        {
            // Exclude loopback interfaces and inactive interfaces
            if (interface.flags() & QNetworkInterface::IsLoopBack || !(interface.flags() & QNetworkInterface::IsUp))
                continue;

            QList<QNetworkAddressEntry> addresses = interface.addressEntries();
            foreach (const QNetworkAddressEntry &address, addresses)
            {
                if (address.ip().protocol() == QAbstractSocket::IPv4Protocol)
                {
                    QString localIpAddress = address.ip().toString();
                    Logger::Log("Local IP Address:" + localIpAddress.toStdString(), LogLevel::INFO, DeviceType::MAIN);

                    if (!localIpAddress.isEmpty())
                    {
                        QUrl getUrl(QStringLiteral("ws://%1:8600").arg(localIpAddress));
                        QUrl getUrlHttps(QStringLiteral("wss://%1:8601").arg(localIpAddress));
                        // Logger::Log("WebSocket URL:" + getUrl.toString().toStdString(), LogLevel::INFO, DeviceType::MAIN);
                        websockethttpUrl = getUrl;
                        websockethttpsUrl = getUrlHttps;
                        found = true;
                        break;
                    }
                    else
                    {
                        Logger::Log("Failed to get local IP address.", LogLevel::WARNING, DeviceType::MAIN);
                    }
                }
            }
            if (found)
                break;
        }

        if (found)
            break;

        retryCount++;
        QThread::sleep(waitTime / 1000); // wait 5 seconds
    }

    if (retryCount == maxRetries)
    {
        qCritical() << "Failed to detect any network interfaces after" << maxRetries << "attempts.";
    }
}

void MainWindow::onMessageReceived(const QString &message)
{
    // Process received messages
    Logger::Log("Received message in MainWindow:" + message.toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
    
    // Split messages to extract commands (for subsequent processing)
    QStringList parts = message.split(':');
    QString command = parts.size() > 0 ? parts[0].trimmed() : message.trimmed();
    
    // Anti-shake check: If the exact same command (including parameters) as the last one is received within a short period of time, only one will be executed.
    // Only keep the last command and only check whether the last command is repeated.
    QString trimmedMessage = message.trimmed();
    qint64 currentTime = QDateTime::currentMSecsSinceEpoch();
    
    // Check if the current command is the same as the last command and within the time window
    if (!lastCommandMessage.isEmpty() && lastCommandMessage == trimmedMessage && lastCommandTime > 0)
    {
        qint64 timeDiff = currentTime - lastCommandTime;
        
        if (timeDiff < COMMAND_DEBOUNCE_MS)
        {
            // When the same command as the last one is received within the time window (the command and parameters are the same), execution is skipped
            Logger::Log("Command debounce: Skipping duplicate message '" + trimmedMessage.toStdString() + 
                       "' received within " + std::to_string(timeDiff) + "ms (threshold: " + 
                       std::to_string(COMMAND_DEBOUNCE_MS) + "ms)", LogLevel::DEBUG, DeviceType::MAIN);
            return;
        }
    }
    
    // Update last command and timestamp
    lastCommandMessage = trimmedMessage;
    lastCommandTime = currentTime;
    
    // split message
    // QStringList parts = message.split(':');

    if (parts.size() >= 2 && parts[0].trimmed() == "ConfirmIndiDriver")
    {
        if (parts.size() == 2)
        {
            Logger::Log("ConfirmIndiDriver:" + parts[1].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
            QString driverName = parts[1].trimmed();
            indi_Driver_Confirm(driverName, "9600");
        }
        else if (parts.size() == 3)
        {
            Logger::Log("ConfirmIndiDriver:" + parts[1].trimmed().toStdString() + ":" + parts[2].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
            QString driverName = parts[1].trimmed();
            QString baudRate = parts[2].trimmed();
            indi_Driver_Confirm(driverName, baudRate);
        }
    }
    else if (message == "ClearIndiDriver")
    {
        Logger::Log("ClearIndiDriver", LogLevel::DEBUG, DeviceType::MAIN);
        indi_Driver_Clear();
    }
    else if (parts.size() == 3 && parts[0].trimmed() == "ConfirmIndiDevice")
    {
        Logger::Log("ConfirmIndiDevice:" + parts[1].trimmed().toStdString() + ":" + parts[2].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        QString deviceName = parts[1].trimmed();
        QString driverName = parts[2].trimmed();
        // connectDevice(x);
        indi_Device_Confirm(deviceName, driverName);
    }
    else if (parts.size() == 3 && parts[0].trimmed() == "BindingDevice")
    {
        Logger::Log("BindingDevice:" + parts[1].trimmed().toStdString() + ":" + parts[2].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        QString devicetype = parts[1].trimmed();
        int deviceindex = parts[2].trimmed().toInt();
        // connectDevice(x);
        BindingDevice(devicetype, deviceindex);
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "UnBindingDevice")
    {
        Logger::Log("UnBindingDevice:" + parts[1].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        QString devicetype = parts[1].trimmed();
        // connectDevice(x);
        UnBindingDevice(devicetype);
    }
    else if (parts.size() == 3 && parts[0].trimmed() == "SelectIndiDriver")
    {
        Logger::Log("SelectIndiDriver:" + parts[1].trimmed().toStdString() + ":" + parts[2].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        QString Group = parts[1].trimmed();
        int ListNum = parts[2].trimmed().toInt();
        printDevGroups2(drivers_list, ListNum, Group);
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "takeExposure")
    {
        Logger::Log("takeExposure:" + parts[1].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::CAMERA);
        Logger::Log("Accept takeExposure order ,set ExpTime is " + parts[1].trimmed().toStdString() + " ms", LogLevel::DEBUG, DeviceType::CAMERA);
        int ExpTime = parts[1].trimmed().toInt();
        INDI_Capture(ExpTime);
        glExpTime = ExpTime;
    }else if (parts.size() == 2 && parts[0].trimmed() == "setExposureTime")
    {
        Logger::Log("setExposureTime:" + parts[1].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::CAMERA);
        int ExpTime = parts[1].trimmed().toInt();
        glExpTime = ExpTime;  // Set exposure time (ms)
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "focusSpeed")
    {
        Logger::Log("change focuser Speed to:" + parts[1].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::FOCUSER);
        int Speed = parts[1].trimmed().toInt();
        // qDebug() << Speed;
        int Speed_ = FocuserControl_setSpeed(Speed);
        emit wsThread->sendMessageToClient("FocusChangeSpeedSuccess:" + QString::number(Speed_));
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "focusMove")
    {
        Logger::Log("focuser to " + parts[1].trimmed().toStdString() + " move ", LogLevel::DEBUG, DeviceType::FOCUSER);
        QString LR = parts[1].trimmed();
        // int Steps = parts[2].trimmed().toInt();
        if (LR == "Left")
        {
            Logger::Log("focuser to Left move ", LogLevel::INFO, DeviceType::FOCUSER);
            FocuserControlMove(true);
            // FocusMoveAndCalHFR(true,Steps);
        }
        else if (LR == "Right")
        {
            Logger::Log("focuser to Right move ", LogLevel::INFO, DeviceType::FOCUSER);
            // FocusMoveAndCalHFR(false,Steps);
            FocuserControlMove(false);
        }
        // else if(LR == "Target")
        // {
        //     FocusGotoAndCalFWHM(Steps);
        // }
    }else if (parts.size() == 3 && parts[0].trimmed() == "focusMoveStep")
    {
        Logger::Log("focuser to " + parts[1].trimmed().toStdString() + " move " + parts[2].trimmed().toStdString() + " steps", LogLevel::DEBUG, DeviceType::FOCUSER);
        QString LR = parts[1].trimmed();
        int Steps = parts[2].trimmed().toInt();
        // When stepping, allow a new step to be executed immediately if the previous move has completed
        // Note: The anti-shake mechanism will block the exact same command, but a command with a different number of syncs should be able to be executed
        FocuserControlMoveStep(LR == "Left", Steps);
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "getFocuserMoveState")
    {
        if (isFocusMoveDone && parts[1].trimmed() == "true")
        {
            focusMoveEndTime = 2;
        }
    }
    else if (parts[0].trimmed() == "focusMoveStop" && parts.size() == 2)
    {
        if (parts[1].trimmed() == "false")
        {
            FocuserControlStop(false);
        }
        else if (parts[1].trimmed() == "true")
        {
            FocuserControlStop(true);
        }
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "SyncFocuserStep")
    {
        Logger::Log("SyncFocuserStep:" + parts[1].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        int Steps = parts[1].trimmed().toInt();
        if (dpFocuser != NULL)
        {
            indi_Client->syncFocuserPosition(dpFocuser, Steps);
            sleep(1);
            CurrentPosition = FocuserControl_getPosition();
            Logger::Log("Focuser Current Position: " + std::to_string(CurrentPosition), LogLevel::DEBUG, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("FocusPosition:" + QString::number(CurrentPosition) + ":" + QString::number(CurrentPosition));
        }
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "MinLimit")
    {
        Logger::Log("MinLimit:" + parts[1].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        int MinLimit = parts[1].trimmed().toInt();
        if (dpFocuser != NULL)
        {
            Tools::saveParameter("Focuser", "focuserMinPosition", parts[1].trimmed());
            focuserMinPosition = MinLimit;
        }
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "MaxLimit")
    {
        Logger::Log("MaxLimit:" + parts[1].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        int MaxLimit = parts[1].trimmed().toInt();
        if (dpFocuser != NULL)
        {
            Tools::saveParameter("Focuser", "focuserMaxPosition", parts[1].trimmed());
            focuserMaxPosition = MaxLimit;
        }
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "Backlash")
    {
        Logger::Log("Backlash:" + parts[1].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        int Backlash = parts[1].trimmed().toInt();
        if (dpFocuser != NULL)
        {
            Tools::saveParameter("Focuser", "Backlash", parts[1].trimmed());
            autofocusBacklashCompensation = Backlash;
        }
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "Coarse Step Divisions")
    {
        Logger::Log("Coarse Step Divisions:" + parts[1].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        int divisions = parts[1].trimmed().toInt();
        if (divisions <= 0)
        {
            divisions = 10;
        }
        autoFocusCoarseDivisions = divisions;
        Tools::saveParameter("Focuser", "coarseStepDivisions", QString::number(divisions));
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "AutoFocus Exposure Time (ms)")
    {
        QString valueStr = parts[1].trimmed();
        int exposureMs = valueStr.toInt();
        if (exposureMs <= 0)
        {
            exposureMs = 1000;
        }

        autoFocusExposureTime = exposureMs;

        Logger::Log("AutoFocus Exposure Time (ms) is set to " + std::to_string(autoFocusExposureTime),
                    LogLevel::DEBUG, DeviceType::MAIN);
        Tools::saveParameter("Focuser", "AutoFocusExposureTime(ms)", valueStr);

        if (autoFocus != nullptr)
        {
            autoFocus->setDefaultExposureTime(autoFocusExposureTime);
        }
    }
    else if (parts.size() == 3 && parts[0].trimmed() == "setROIPosition")
    {
        Logger::Log("setROIPosition:" + parts[1].trimmed().toStdString() + "*" + parts[2].trimmed().toStdString(), LogLevel::INFO, DeviceType::MAIN);
        roiAndFocuserInfo["ROI_x"] = parts[1].trimmed().toDouble();
        roiAndFocuserInfo["ROI_y"] = parts[2].trimmed().toDouble();
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "RedBoxSizeChange")
    {
        Logger::Log("RedBoxSizeChange:" + parts[1].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        BoxSideLength = parts[1].trimmed().toInt();
        roiAndFocuserInfo["BoxSideLength"] = BoxSideLength;
        Tools::saveParameter("MainCamera", "RedBoxSize", parts[1].trimmed());
        Logger::Log("RedBoxSizeChange:" + std::to_string(BoxSideLength), LogLevel::DEBUG, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("MainCameraSize:" + QString::number(glMainCCDSizeX) + ":" + QString::number(glMainCCDSizeY));
    }

    else if (message.startsWith("AutoFocusConfirm:")) // [AUTO_FOCUS_UI_ENHANCEMENT]
    {
        const QString prefix = "AutoFocusConfirm:";
        QString mode = message.mid(prefix.length()).trimmed();

        // Send autofocus start event to the front end
        if (mode.compare("No", Qt::CaseInsensitive) != 0) {
            if (isAutoFocus) {
                Logger::Log("AutoFocus already started", LogLevel::INFO, DeviceType::MAIN);
                return;
            }
        }

        if (mode.isEmpty() || mode == "Yes" || mode == "Coarse") {
            // Complete autofocus process: coarse adjustment + fine adjustment + super-fine
            // Add mode mark: full to facilitate the front-end to distinguish the UI behavior of different autofocus modes
            emit wsThread->sendMessageToClient("AutoFocusStarted:full:Autofocus has started");
            startAutoFocus();
        }
        else if (mode == "Fine") {
            // New: Perform HFR fine tuning only from current position (fixed step size 100, samples 11 points)
            // Add mode flag: fine (only fine tuning mode)
            emit wsThread->sendMessageToClient("AutoFocusStarted:fine:Autofocus has started");
            startAutoFocusFineHFROnly();
        }
        else { // No or unknown mode, considered canceled
            Logger::Log("User cancels autofocus", LogLevel::INFO, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("AutoFocusCancelled:User has canceled autofocus");
        }
    }
    else if (message == "StopAutoFocus")
    {
        if (!isAutoFocus) {
            Logger::Log("AutoFocus not started, stopAutoFocus failed", LogLevel::INFO, DeviceType::MAIN);
            return;
        }
        Logger::Log("StopAutoFocus", LogLevel::DEBUG, DeviceType::MAIN);
        isAutoFocus = false;
        autoFocus->stopAutoFocus();
        autoFocus->deleteLater();
        cleanupAutoFocusConnections();
        autoFocus = nullptr;
        emit wsThread->sendMessageToClient("AutoFocusEnded:Autofocus ended");
    }
    else if (message == "abortExposure")
    {
        Logger::Log("abortExposure", LogLevel::DEBUG, DeviceType::CAMERA);
        INDI_AbortCapture();
    }
    else if (message == "RestartPHD2" || message == "PHD2RestartConfirm")
    {
        Logger::Log("Frontend requested to restart PHD2", LogLevel::INFO, DeviceType::GUIDER);
        // Disconnect the guide device first (all are existing interfaces, no new functions are added)
        if (dpGuider && dpGuider->isConnected()) {
            DisconnectDevice(indi_Client, dpGuider->getDeviceName(), "Guider");
        }
        // Restart PHD2
        InitPHD2();
        emit wsThread->sendMessageToClient("PHD2Restarting");
    }
    else if (message == "connectAllDevice")
    {
        Logger::Log("connectAllDevice", LogLevel::DEBUG, DeviceType::MAIN);
        // DeviceConnect();
        ConnectAllDeviceOnce();
    }
    else if (message == "disconnectAllDevice")
    {
        Logger::Log("disconnectAllDevice ...", LogLevel::DEBUG, DeviceType::MAIN);
        disconnectIndiServer(indi_Client);
        Logger::Log("disconnectIndiServer ...", LogLevel::DEBUG, DeviceType::MAIN);
        // ClearSystemDeviceList();
        // Logger::Log("ClearSystemDeviceList ...", LogLevel::DEBUG, DeviceType::MAIN);
        clearConnectedDevices();
        Logger::Log("clearConnectedDevices ...", LogLevel::DEBUG, DeviceType::MAIN);
        // Restart INDI server
        initINDIServer();
        Logger::Log("initINDIServer ...", LogLevel::DEBUG, DeviceType::MAIN);
        initINDIClient();

        Logger::Log("initINDIClient ...", LogLevel::DEBUG, DeviceType::MAIN);
        // Tools::InitSystemDeviceList();
        // Logger::Log("Tools::InitSystemDeviceList ...", LogLevel::DEBUG, DeviceType::MAIN);
        // Tools::initSystemDeviceList(systemdevicelist);
        getLastSelectDevice();
        Logger::Log("disconnectAllDevice end!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (message == "MountMoveWest")
    {
        Logger::Log("MountMoveWest ...", LogLevel::DEBUG, DeviceType::MOUNT);
        if (dpMount != NULL)
        {
            indi_Client->setTelescopeMoveWE(dpMount, "WEST");
        }
    }
    else if (message == "MountMoveEast")
    {
        Logger::Log("MountMoveEast ...", LogLevel::DEBUG, DeviceType::MOUNT);
        if (dpMount != NULL)
        {
            indi_Client->setTelescopeMoveWE(dpMount, "EAST");
        }
    }
    else if (message == "MountMoveNorth")
    {
        Logger::Log("MountMoveNorth ...", LogLevel::DEBUG, DeviceType::MOUNT);
        if (dpMount != NULL)
        {
            indi_Client->setTelescopeMoveNS(dpMount, "NORTH");
        }
    }
    else if (message == "MountMoveSouth")
    {
        Logger::Log("MountMoveSouth ...", LogLevel::DEBUG, DeviceType::MOUNT);
        if (dpMount != NULL)
        {
            indi_Client->setTelescopeMoveNS(dpMount, "SOUTH");
        }
    }
    else if (message == "MountMoveAbort")
    {
        Logger::Log("MountMoveAbort ...", LogLevel::DEBUG, DeviceType::MOUNT);
        if (dpMount != NULL)
        {
            indi_Client->setTelescopeAbortMotion(dpMount);
        }
    }else if (parts[0].trimmed() == "MountMoveRAStop")
    {
        Logger::Log("MountMoveRAStop ...", LogLevel::DEBUG, DeviceType::MOUNT);
        if (dpMount != NULL)
        {
            indi_Client->setTelescopeMoveWE(dpMount, "STOP");
        }
    }else if (parts[0].trimmed() == "MountMoveDECStop")
    {
        Logger::Log("MountMoveDECStop ...", LogLevel::DEBUG, DeviceType::MOUNT);
        if (dpMount != NULL)
        {
            indi_Client->setTelescopeMoveNS(dpMount, "STOP");
        }
    }
    else if (message == "MountPark")
    {
        Logger::Log("MountPark ...", LogLevel::DEBUG, DeviceType::MOUNT);
        if (dpMount != NULL)
        {
            bool isPark = TelescopeControl_Park();
            if (isPark)
            {
                emit wsThread->sendMessageToClient("TelescopePark:ON");
            }
            else
            {
                emit wsThread->sendMessageToClient("TelescopePark:OFF");
            }
        }
    }
    else if (message == "MountTrack")
    {
        Logger::Log("MountTrack ...", LogLevel::DEBUG, DeviceType::MOUNT);
        if (dpMount != NULL)
        {
            bool isTrack = TelescopeControl_Track();
            if (isTrack)
            {
                emit wsThread->sendMessageToClient("TelescopeTrack:ON");
                Logger::Log("TelescopeTrack:ON", LogLevel::DEBUG, DeviceType::MOUNT);
            }
            else
            {
                emit wsThread->sendMessageToClient("TelescopeTrack:OFF");
                Logger::Log("TelescopeTrack:OFF", LogLevel::DEBUG, DeviceType::MOUNT);
            }
        }
    }
    else if (message == "MountHome")
    {
        Logger::Log("MountHome ...", LogLevel::DEBUG, DeviceType::MOUNT);
        if (dpMount != NULL)
        {
            indi_Client->setTelescopeHomeInit(dpMount, "SLEWHOME");
        }
    }
    else if (message == "MountSYNC")
    {
        Logger::Log("MountSYNC ...", LogLevel::DEBUG, DeviceType::MOUNT);
        if (dpMount != NULL)
        {
            indi_Client->setTelescopeHomeInit(dpMount, "SYNCHOME");
        }
    }

    else if (message == "MountSpeedSwitch")
    {
        Logger::Log("MountSpeedSwitch ", LogLevel::DEBUG, DeviceType::MOUNT);
        if (dpMount != NULL)
        {
            int currentSpeed;
            indi_Client->getTelescopeSlewRate(dpMount, currentSpeed);
            Logger::Log("Current Speed:" + std::to_string(currentSpeed) + " Total Speed:" + std::to_string(glTelescopeTotalSlewRate), LogLevel::DEBUG, DeviceType::MOUNT);

            if (currentSpeed == glTelescopeTotalSlewRate)
            {
                indi_Client->setTelescopeSlewRate(dpMount, 1);
            }
            else
            {
                indi_Client->setTelescopeSlewRate(dpMount, currentSpeed + 1);
                Logger::Log("Set Speed to:" + std::to_string(currentSpeed), LogLevel::DEBUG, DeviceType::MOUNT);
            }

            int ChangedSpeed;
            indi_Client->getTelescopeSlewRate(dpMount, ChangedSpeed);
            Logger::Log("Changed Speed:" + std::to_string(ChangedSpeed), LogLevel::DEBUG, DeviceType::MOUNT);
            emit wsThread->sendMessageToClient("MountSetSpeedSuccess:" + QString::number(ChangedSpeed));
        }
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "GotoThenSolve")
    {
        Logger::Log("GotoThenSolve ...", LogLevel::DEBUG, DeviceType::MOUNT);
        if (parts[1].trimmed() == "true")
        {
            GotoThenSolve = true;
        }
        else
        {
            GotoThenSolve = false;
        }
        Tools::saveParameter("Mount", "GotoThenSolve", parts[1].trimmed());
    }else if (parts.size() == 3 && parts[0].trimmed() == "Goto")
    {
        Logger::Log("Goto ...", LogLevel::DEBUG, DeviceType::MOUNT);
        double Ra = parts[1].trimmed().toDouble();
        double Dec = parts[2].trimmed().toDouble();
        MountOnlyGoto(Ra, Dec);
        Logger::Log("Goto finish!", LogLevel::DEBUG, DeviceType::MOUNT);
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "AutoFlip")
    {
        isAutoFlip = parts[1].trimmed() == "true" ? true : false;
        Tools::saveParameter("Mount", "AutoFlip", parts[1].trimmed());
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "EastMinutesPastMeridian")
    {
        EastMinutesPastMeridian = parts[1].trimmed().toDouble();
        Tools::saveParameter("Mount", "EastMinutesPastMeridian", parts[1].trimmed());
        if (dpMount != NULL)
        {
            indi_Client->setMinutesPastMeridian(dpMount, EastMinutesPastMeridian, WestMinutesPastMeridian);
            // emit wsThread->sendMessageToClient("MinutesPastMeridian:" + QString::number(EastMinutesPastMeridian) + ":" + QString::number(WestMinutesPastMeridian));
        }
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "WestMinutesPastMeridian")
    {
        WestMinutesPastMeridian = parts[1].trimmed().toDouble();
        Tools::saveParameter("Mount", "WestMinutesPastMeridian", parts[1].trimmed());
        if (dpMount != NULL)
        {
            indi_Client->setMinutesPastMeridian(dpMount, EastMinutesPastMeridian, WestMinutesPastMeridian);
            // emit wsThread->sendMessageToClient("MinutesPastMeridian:" + QString::number(EastMinutesPastMeridian) + ":" + QString::number(WestMinutesPastMeridian));
        }
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "ImageGainR")
    {
        ImageGainR = parts[1].trimmed().toDouble();
        Logger::Log("GainR is set to " + std::to_string(ImageGainR), LogLevel::DEBUG, DeviceType::MAIN);
        Tools::saveParameter("MainCamera", "ImageGainR", parts[1].trimmed());
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "ImageGainB")
    {
        ImageGainB = parts[1].trimmed().toDouble();
        Logger::Log("GainB is set to " + std::to_string(ImageGainB), LogLevel::DEBUG, DeviceType::MAIN);
        Tools::saveParameter("MainCamera", "ImageGainB", parts[1].trimmed());
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "ImageOffset")
    {
        ImageOffset = parts[1].trimmed().toDouble();
        Logger::Log("ImageOffset is set to " + std::to_string(ImageOffset), LogLevel::DEBUG, DeviceType::MAIN);
        Tools::saveParameter("MainCamera", "Offset", parts[1].trimmed());
        if (dpMainCamera != NULL)
        {
            indi_Client->setCCDOffset(dpMainCamera, ImageOffset);
        }
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "ImageCFA")
    {
        MainCameraCFA = parts[1].trimmed();
        Logger::Log("ImageCFA is set to " + MainCameraCFA.toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        Tools::saveParameter("MainCamera", "ImageCFA", parts[1].trimmed());
    }
    else if (parts.size() == 3 && parts[0].trimmed() == "SetSerialPort")
    {
        // Manually set the serial port path, only for Mount/Focuser
        QString devType = parts[1].trimmed();
        QString portPath = parts[2].trimmed();

        // Special value "default" or empty string: means returning to automatic matching mode and not forcing the serial port to be specified.
        bool isDefault =
            portPath.trimmed().isEmpty() ||
            portPath.trimmed().compare("default", Qt::CaseInsensitive) == 0;

        if (devType == "Mount")
        {
            mountSerialPortOverride = isDefault ? QString() : portPath;
        }
        else if (devType == "Focuser")
        {
            focuserSerialPortOverride = isDefault ? QString() : portPath;
        }

        if (isDefault)
        {
            Logger::Log("SetSerialPort | " + devType.toStdString() + " -> <default(auto-detect)>",
                        LogLevel::INFO, DeviceType::MAIN);
            // The device port is not modified immediately, and automatic identification/rematching logic is used during subsequent connections.
            return;
        }

        Logger::Log("SetSerialPort | " + devType.toStdString() + " -> " + portPath.toStdString(),
                    LogLevel::INFO, DeviceType::MAIN);

        // If the device already exists, its serial port will be updated immediately (only the memory status will be modified, not persisted)
        if (devType == "Mount" && dpMount != nullptr)
        {
            indi_Client->setDevicePort(dpMount, portPath);
        }
        else if (devType == "Focuser" && dpFocuser != nullptr)
        {
            indi_Client->setDevicePort(dpFocuser, portPath);
        }
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "Self Exposure Time (ms)")
    {
        Logger::Log("Self Exposure Time (ms) is set to " + parts[1].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        Tools::saveParameter("MainCamera", "SelfExposureTime(ms)", parts[1].trimmed());
    }
    

    else if (parts[0].trimmed() == "ScheduleTabelData")
    {
        ScheduleTabelData(message);
    }
    else if (parts.size() == 4 && parts[0].trimmed() == "MountGoto")
    {
        if (dpMount != NULL)
        {
            Logger::Log("MountGoto ...", LogLevel::DEBUG, DeviceType::MOUNT);
            QStringList RaDecList = message.split(',');
            QStringList RaList = RaDecList[0].split(':');
            QStringList DecList = RaDecList[1].split(':');

            double Ra_Rad, Dec_Rad;
            Ra_Rad = RaList[2].trimmed().toDouble();
            Dec_Rad = DecList[1].trimmed().toDouble();

            Logger::Log("Mount Goto RaDec(Rad):" + std::to_string(Ra_Rad) + "," + std::to_string(Dec_Rad), LogLevel::DEBUG, DeviceType::MOUNT);

            double Ra_Hour, Dec_Degree;
            Ra_Hour = Tools::RadToHour(Ra_Rad);
            Dec_Degree = Tools::RadToDegree(Dec_Rad);

            MountGoto(Ra_Hour, Dec_Degree);
            Logger::Log("MountGoto finish!", LogLevel::DEBUG, DeviceType::MOUNT);
        }
        else
        {
        }
    }
    else if (message == "StopSchedule")
    {
        Logger::Log("StopSchedule !", LogLevel::DEBUG, DeviceType::MAIN);
    StopSchedule = true;
    isScheduleRunning = false;
    // Immediately notify the front-end that the scheduled task has stopped, avoiding the delays caused by relying solely on progress inference
    emit wsThread->sendMessageToClient("ScheduleRunning:false");
        
        // Also stop autofocus if it is running (especially if triggered by a scheduler)
        if (isAutoFocus && autoFocus != nullptr)
        {
            Logger::Log("When stopping the scheduled task list, it is detected that the autofocus is running and the autofocus is stopped at the same time.", LogLevel::INFO, DeviceType::MAIN);
            isScheduleTriggeredAutoFocus = false; // Clear the flag to avoid continuing to shoot after autofocus is completed
            autoFocus->stopAutoFocus();
            cleanupAutoFocusConnections();
            autoFocus->deleteLater();
            autoFocus = nullptr;
            isAutoFocus = false;
            emit wsThread->sendMessageToClient("AutoFocusEnded:Autofocus has stopped (schedule has been paused)");
        }
        
        // Immediately stop the exposure delay timer
        bool wasActive = exposureDelayTimer.isActive();
        exposureDelayTimer.stop();
        exposureDelayTimer.disconnect();
        if (wasActive || exposureDelayElapsed_ms > 0)
        {
            Logger::Log(("Exposure delay timer stopped immediately (wasActive: " + QString::number(wasActive ? 1 : 0) + ", elapsed: " + QString::number(exposureDelayElapsed_ms) + " ms)").toStdString(), LogLevel::INFO, DeviceType::MAIN);
            qDebug() << "Exposure delay timer stopped immediately (wasActive:" << wasActive << ", elapsed:" << exposureDelayElapsed_ms << "ms)";
            exposureDelayElapsed_ms = 0; // Reset elapsed time
        }
    }

    else if (message == "CaptureImageSave")
    {
        Logger::Log("CaptureImageSave ...", LogLevel::DEBUG, DeviceType::MAIN);
        CaptureImageSave();
        Logger::Log("CaptureImageSave finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (message == "getClientSettings")
    {
        Logger::Log("getClientSettings ...", LogLevel::DEBUG, DeviceType::MAIN);
        getClientSettings();
        Logger::Log("getClientSettings finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (message == "getConnectedDevices")
    {
        // getConnectedDevices();
    }

    else if (message == "getStagingImage")
    {
        Logger::Log("Reload Main Camera Image.", LogLevel::DEBUG, DeviceType::MAIN);
        getStagingImage();
        Logger::Log("Reload Main Camera Image finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (parts[0].trimmed() == "StagingScheduleData")
    {
        Logger::Log("StagingScheduleData ...", LogLevel::DEBUG, DeviceType::MAIN);
        isStagingScheduleData = true;
        StagingScheduleData = message;
        Logger::Log("StagingScheduleData finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (message == "getStagingScheduleData")
    {
        Logger::Log("getStagingScheduleData ...", LogLevel::DEBUG, DeviceType::MAIN);
        getStagingScheduleData();
        Logger::Log("getStagingScheduleData finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    // ----------Schedule presets (task schedule preset management) ----------
    else if (parts[0].trimmed() == "saveSchedulePreset" && parts.size() >= 3)
    {
        // Format: save schedule preset:<name>:<raw data>
        QString presetName = parts[1].trimmed();
        // Re-splice data (to prevent data containing ':' from being split)
        QString rawData;
        for (int i = 2; i < parts.size(); ++i)
        {
            if (i > 2)
                rawData += ":";
            rawData += parts[i];
        }

        Logger::Log("saveSchedulePreset | name=" + presetName.toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        Tools::saveSchedulePreset(presetName, rawData);
    }
    else if (parts[0].trimmed() == "loadSchedulePreset" && parts.size() == 2)
    {
        // Format: load schedule preset:<name>
        QString presetName = parts[1].trimmed();
        Logger::Log("loadSchedulePreset | name=" + presetName.toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        QString data = Tools::readSchedulePreset(presetName);
        if (!data.isEmpty())
        {
            // Reuse the existing StagingScheduleData protocol and push it directly to the front end
            QString messageOut = "StagingScheduleData:" + data;
            emit wsThread->sendMessageToClient(messageOut);
        }
    }
    else if (parts[0].trimmed() == "deleteSchedulePreset" && parts.size() == 2)
    {
        // Format: delete schedule preset:<name>
        QString presetName = parts[1].trimmed();
        Logger::Log("deleteSchedulePreset | name=" + presetName.toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        bool ok = Tools::deleteSchedulePreset(presetName);
        if (!ok)
        {
            Logger::Log("deleteSchedulePreset | failed to delete preset: " + presetName.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
        }
    }
    else if (message == "listSchedulePresets")
    {
        // Return format: schedule preset list:name1;name2;name3
        Logger::Log("listSchedulePresets ...", LogLevel::DEBUG, DeviceType::MAIN);
        QStringList names = Tools::listSchedulePresets();
        QString payload = "SchedulePresetList:";
        for (int i = 0; i < names.size(); ++i)
        {
            if (i > 0)
                payload += ";";
            payload += names.at(i);
        }
        emit wsThread->sendMessageToClient(payload);
        Logger::Log("listSchedulePresets finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (message == "getStagingGuiderData")
    {
        Logger::Log("getStagingGuiderData ...", LogLevel::DEBUG, DeviceType::MAIN);
        getStagingGuiderData();
        Logger::Log("getStagingGuiderData finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (parts[0].trimmed() == "ExpTimeList")
    {
        Logger::Log("ExpTimeList ...", LogLevel::DEBUG, DeviceType::MAIN);
        Tools::saveExpTimeList(message);
        Logger::Log("ExpTimeList finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (message == "getExpTimeList")
    {
        Logger::Log("getExpTimeList ...", LogLevel::DEBUG, DeviceType::MAIN);
        if (Tools::readExpTimeList() != QString())
        {
            emit wsThread->sendMessageToClient(Tools::readExpTimeList());
        }
        Logger::Log("getExpTimeList finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (message == "getCaptureStatus")
    {
        Logger::Log("getCaptureStatus ...", LogLevel::DEBUG, DeviceType::MAIN);
        if (glMainCameraStatu == "Exposuring")
        {
            if (isFocusLoopShooting)
            {
                emit wsThread->sendMessageToClient("CameraInExposuring:False");
            }
            else
            {
                if (isFocusLoopShooting)
                {
                    emit wsThread->sendMessageToClient("CameraInExposuring:False");
                }
                else
                {
                    emit wsThread->sendMessageToClient("CameraInExposuring:True");
                }
            }
        }
        Logger::Log("getCaptureStatus finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (parts[0].trimmed() == "SetCFWPosition" && parts.size() == 2)
    {
        Logger::Log("SetCFWPosition ...", LogLevel::DEBUG, DeviceType::CFW);
        int pos = parts[1].trimmed().toInt();

        if (isFilterOnCamera)
        {
            if (dpMainCamera != NULL)
            {
                indi_Client->setCFWPosition(dpMainCamera, pos);
                emit wsThread->sendMessageToClient("SetCFWPositionSuccess:" + QString::number(pos));
                Logger::Log("Set CFW Position to" + std::to_string(pos) + "Success!!!", LogLevel::DEBUG, DeviceType::CFW);
            }
        }
        else
        {
            if (dpCFW != NULL)
            {
                indi_Client->setCFWPosition(dpCFW, pos);
                emit wsThread->sendMessageToClient("SetCFWPositionSuccess:" + QString::number(pos));
                Logger::Log("Set CFW Position to" + std::to_string(pos) + "Success!!!", LogLevel::DEBUG, DeviceType::CFW);
            }
        }
    }

    else if (parts[0].trimmed() == "CFWList")
    {
        Logger::Log("Save CFWList ...", LogLevel::DEBUG, DeviceType::CFW);
        if (isFilterOnCamera)
        {
            if (dpMainCamera != NULL)
            {
                QString CFWname;
                indi_Client->getCFWSlotName(dpMainCamera, CFWname);
                Tools::saveCFWList(CFWname, parts[1]);
            }
        }
        else
        {
            if (dpCFW != NULL)
            {
                Tools::saveCFWList(QString::fromUtf8(dpCFW->getDeviceName()), parts[1]);
            }
        }
        Logger::Log("Save CFWList finish!", LogLevel::DEBUG, DeviceType::CFW);
    }

    else if (message == "getCFWList")
    {
        Logger::Log("get CFWList ...", LogLevel::DEBUG, DeviceType::CFW);
        if (isFilterOnCamera)
        {
            if (dpMainCamera != NULL)
            {
                // int min, max, pos;
                // indi_Client->getCFWPosition(dpMainCamera, pos, min, max);
                // emit wsThread->sendMessageToClient("CFWPositionMax:" + QString::number(max));
                QString CFWname;
                indi_Client->getCFWSlotName(dpMainCamera, CFWname);
                if (Tools::readCFWList(CFWname) != QString())
                {
                    emit wsThread->sendMessageToClient("getCFWList:" + Tools::readCFWList(CFWname));
                }
            }
        }
        else
        {
            if (dpCFW != NULL)
            {
                // int min, max, pos;
                // indi_Client->getCFWPosition(dpCFW, pos, min, max);
                // emit wsThread->sendMessageToClient("CFWPositionMax:" + QString::number(max));
                if (Tools::readCFWList(QString::fromUtf8(dpCFW->getDeviceName())) != QString())
                {
                    emit wsThread->sendMessageToClient("getCFWList:" + Tools::readCFWList(QString::fromUtf8(dpCFW->getDeviceName())));
                }
            }
        }
        Logger::Log("get CFWList finish!", LogLevel::DEBUG, DeviceType::CFW);
    }

    else if (message == "ClearCalibrationData")
    {
        ClearCalibrationData = true;
        Logger::Log("Clear polar alignment calibration data", LogLevel::DEBUG, DeviceType::MAIN);
    }else if (message == "getGuiderStatus")
    {
        Logger::Log("getGuiderStatus ...", LogLevel::DEBUG, DeviceType::GUIDER);
        if (isGuiding && dpGuider != NULL)
        {
            emit wsThread->sendMessageToClient("GuiderSwitchStatus:true");
        }
        else
        {
            emit wsThread->sendMessageToClient("GuiderSwitchStatus:false");
        }
        if ( isGuiderLoopExp && dpGuider != NULL)
        {
            emit wsThread->sendMessageToClient("GuiderLoopExpStatus:true");
        }
        else
        {
            emit wsThread->sendMessageToClient("GuiderLoopExpStatus:false");
        }
        
        Logger::Log("getGuiderStatus finish!", LogLevel::DEBUG, DeviceType::GUIDER);
    }

    else if (parts[0].trimmed() == "GuiderSwitch" && parts.size() == 2)
    {
        Logger::Log("GuiderSwitch ...", LogLevel::INFO, DeviceType::GUIDER);
        if (isGuiding && parts[1].trimmed() == "false")
        {

            isGuiding = false;
            call_phd_StopLooping();
            emit wsThread->sendMessageToClient("GuiderSwitchStatus:false");
            isGuiderLoopExp = false;
            emit wsThread->sendMessageToClient("GuiderLoopExpStatus:false");
            emit wsThread->sendMessageToClient("GuiderUpdateStatus:0");
            // emit wsThread->sendMessageToClient("GuiderStatus:InCalibration");
            Logger::Log("Stop GuiderSwitch finish!", LogLevel::INFO, DeviceType::GUIDER);
        }
        else if (!isGuiding && parts[1].trimmed() == "true")
        {

            isGuiding = true;
            emit wsThread->sendMessageToClient("GuiderSwitchStatus:true");
            if (ClearCalibrationData)
            {
                ClearCalibrationData = false;
                call_phd_ClearCalibration();
            }
            Logger::Log("clear calibration data finish!", LogLevel::INFO, DeviceType::GUIDER);

            // call_phd_StartLooping();
            // sleep(1);
            if (glPHD_isSelected == false)
            {
                Logger::Log("AutoFindStar is not selected, start AutoFindStar ...", LogLevel::INFO, DeviceType::GUIDER);
                call_phd_AutoFindStar();
            }
            call_phd_StartGuiding();
            emit wsThread->sendMessageToClient("GuiderUpdateStatus:1");
            Logger::Log("Start GuiderSwitch finish!", LogLevel::INFO, DeviceType::GUIDER);
        }
        else
        {
            if (parts[1].trimmed() == "true")
            {
                emit wsThread->sendMessageToClient("GuiderSwitchStatus:true");
                emit wsThread->sendMessageToClient("GuiderUpdateStatus:1");
            }
            else
            {
                emit wsThread->sendMessageToClient("GuiderSwitchStatus:false");
                emit wsThread->sendMessageToClient("GuiderUpdateStatus:0");
            }
            Logger::Log("GuiderSwitch status already set " + parts[1].trimmed().toStdString() + " not change!!!", LogLevel::WARNING, DeviceType::GUIDER);
        }
    }

    else if (parts[0].trimmed() == "GuiderLoopExpSwitch" && parts.size() == 2)
    {
        if (dpGuider != NULL)
        {
            if (isGuiderLoopExp && parts[1].trimmed() == "false")
            {
                // Turn off cycle exposure: stop high frequency timer
                QMetaObject::invokeMethod(m_threadTimer, "stop", Qt::QueuedConnection);
                QMetaObject::invokeMethod(PHDControlGuide_threadTimer, "stop", Qt::QueuedConnection);

                Logger::Log("Stop GuiderLoopExp ...", LogLevel::INFO, DeviceType::GUIDER);
                isGuiderLoopExp = false;
                isGuiding = false;
                call_phd_StopLooping();
                emit wsThread->sendMessageToClient("GuiderLoopExpStatus:false");
                emit wsThread->sendMessageToClient("GuiderUpdateStatus:0");
                Logger::Log("Stop GuiderLoopExp finish!", LogLevel::INFO, DeviceType::GUIDER);
            }
            else if (!isGuiderLoopExp && parts[1].trimmed() == "true")
            {
                Logger::Log("Start GuiderLoopExp ...", LogLevel::INFO, DeviceType::GUIDER);
                isGuiderLoopExp = true;
                // Turn on cycle exposure: start high-frequency timer
                QMetaObject::invokeMethod(m_threadTimer, "start", Qt::QueuedConnection);
                QMetaObject::invokeMethod(PHDControlGuide_threadTimer, "start", Qt::QueuedConnection);

                emit wsThread->sendMessageToClient("GuiderLoopExpStatus:true");
                emit wsThread->sendMessageToClient("GuiderUpdateStatus:1");
                call_phd_StartLooping();
                Logger::Log("Start GuiderLoopExp finish!", LogLevel::INFO, DeviceType::GUIDER);
            }
            else
            {
                Logger::Log("GuiderLoopExp status already set " + parts[1].trimmed().toStdString() + " not change!!!", LogLevel::WARNING, DeviceType::GUIDER);
                if (parts[1].trimmed() == "true")
                {
                    emit wsThread->sendMessageToClient("GuiderLoopExpStatus:true");
                    emit wsThread->sendMessageToClient("GuiderUpdateStatus:1");
                }
                else
                {
                    emit wsThread->sendMessageToClient("GuiderLoopExpStatus:false");
                    emit wsThread->sendMessageToClient("GuiderUpdateStatus:0");
                }
            }
        }
        else
        {
            Logger::Log("GuiderLoopExp is not connected", LogLevel::INFO, DeviceType::GUIDER);
        }
    }

    else if (message == "PHD2Recalibrate")
    {
        Logger::Log("PHD2Recalibrate ...", LogLevel::DEBUG, DeviceType::GUIDER);
        call_phd_ClearCalibration();
        call_phd_StartLooping();

        // Recalibration will also enable cycle exposure. Make sure the high-frequency timer is started.
        QMetaObject::invokeMethod(m_threadTimer, "start", Qt::QueuedConnection);
        QMetaObject::invokeMethod(PHDControlGuide_threadTimer, "start", Qt::QueuedConnection);

        sleep(1);

        call_phd_AutoFindStar();
        call_phd_StartGuiding();
        Logger::Log("PHD2Recalibrate finish!", LogLevel::DEBUG, DeviceType::GUIDER);
        // emit wsThread->sendMessageToClient("GuiderSwitchStatus:true");

        // call_phd_StarClick(641,363);
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "GuiderExpTimeSwitch")
    {
        Logger::Log("GuiderExpTimeSwitch :" + std::to_string(parts[1].toInt()), LogLevel::DEBUG, DeviceType::GUIDER);
        call_phd_setExposureTime(parts[1].toInt());
        Logger::Log("GuiderExpTimeSwitch finish!", LogLevel::DEBUG, DeviceType::GUIDER);
    }

    else if (message == "clearGuiderData")
    {
        Logger::Log("clearGuiderData ...", LogLevel::DEBUG, DeviceType::GUIDER);
        glPHD_rmsdate.clear();
        Logger::Log("clearGuiderData finish!", LogLevel::DEBUG, DeviceType::GUIDER);
    }
    else if (parts[0].trimmed() == "SolveSYNC")
    {
        Logger::Log("SolveSYNC ...", LogLevel::DEBUG, DeviceType::MAIN);
        if (dpMount == NULL ){
            Logger::Log("Mount not connect", LogLevel::DEBUG, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("MountNotConnect");
            return;
        }
        if (dpMainCamera == NULL ){
            Logger::Log("MainCamera not connect", LogLevel::DEBUG, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("MainCameraNotConnect");
            return;
        }
        if (isSolveSYNC)
        {
            Logger::Log("SolveSYNC is already running", LogLevel::DEBUG, DeviceType::MAIN);
            return;
        }
        if (glFocalLength == 0)
        {
            emit wsThread->sendMessageToClient("FocalLengthError");
            Logger::Log("FocalLengthError", LogLevel::DEBUG, DeviceType::MAIN);
        }
        else
        {
            TelescopeControl_SolveSYNC();
            Logger::Log("SolveSYNC finish!", LogLevel::DEBUG, DeviceType::MAIN);
        }
    }

    else if (message == "ClearDataPoints")
    {
        Logger::Log("ClearDataPoints ...", LogLevel::DEBUG, DeviceType::MAIN);
        // FWHM Data
        dataPoints.clear();
        Logger::Log("ClearDataPoints finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (message == "ShowAllImageFolder")
    {
        Logger::Log("ShowAllImageFolder ...", LogLevel::DEBUG, DeviceType::MAIN);
        std::string allFile = GetAllFile();
        emit wsThread->sendMessageToClient("ShowAllImageFolder:" + QString::fromStdString(allFile));
        Logger::Log("ShowAllImageFolder finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (parts.size() >= 2 && parts[0].trimmed() == "MoveFileToUSB")
    {
        Logger::Log("MoveFileToUSB ...", LogLevel::DEBUG, DeviceType::MAIN);
        QStringList ImagePath = parseString(parts[1].trimmed().toStdString(), ImageSaveBasePath);
        QString usbName = "";
        // If the USB disk name is provided (parts.size() >= 3), use it
        if (parts.size() >= 3)
        {
            usbName = parts[2].trimmed();
        }
        RemoveImageToUsb(ImagePath, usbName);
        Logger::Log("MoveFileToUSB finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (parts[0].trimmed() == "DeleteFile")
    {
        Logger::Log("DeleteFile ...", LogLevel::DEBUG, DeviceType::MAIN);
        QString ImagePathString = message; // Create a copy
        ImagePathString.replace("DeleteFile:", "");

        QStringList ImagePath = parseString(ImagePathString.toStdString(), ImageSaveBasePath);
        DeleteImage(ImagePath);
        Logger::Log("DeleteFile finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (message == "USBCheck")
    {
        Logger::Log("USBCheck ...", LogLevel::DEBUG, DeviceType::MAIN);
        USBCheck();
        Logger::Log("USBCheck finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "GetImageFiles")
    {
        Logger::Log("GetImageFiles ...", LogLevel::DEBUG, DeviceType::MAIN);
        std::string FolderPath = parts[1].trimmed().toStdString();
        GetImageFiles(FolderPath);
        Logger::Log("GetImageFiles finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (parts[0].trimmed() == "GetUSBFiles")
    {
        Logger::Log("GetUSBFiles ...", LogLevel::DEBUG, DeviceType::MAIN);
        // Format: GetUSBFiles:usb_name:relativePath
        // Both parameters are required: USB disk name and relative path
        QString usbName = QString();
        QString relativePath = QString();
        
        if (parts.size() >= 3)
        {
            usbName = parts[1].trimmed();
            relativePath = parts[2].trimmed();
        }
        else if (parts.size() >= 2)
        {
            // If there are only two parts, it may be that the old format is compatible, but requires two parameters
            usbName = parts[1].trimmed();
            relativePath = ""; // Empty string, but the function checks internally
        }
        
        // Called directly, the parameters will be verified internally in the get usb files function.
        GetUSBFiles(usbName, relativePath);
        Logger::Log("GetUSBFiles finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (parts[0].trimmed() == "ReadImageFile")
    {
        Logger::Log("ReadImageFile ...", LogLevel::DEBUG, DeviceType::MAIN);
        QString ImagePath = message; // Create a copy
        ImagePath.replace("ReadImageFile:", "image/");
        // ImagePath.replace(" ", "\\ "); //Escape spaces
        // ImagePath.replace("[", "\\["); //Escape the left square bracket
        // ImagePath.replace("]", "\\]"); //Escape the right square bracket
        // ImagePath.replace(",", "\\,"); //escape commas
        saveFitsAsPNG(ImagePath, false);
        Logger::Log("ReadImageFile finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }



    else if (message == "stopLoopSolveImage")
    {
        Logger::Log("stopLoopSolveImage ...", LogLevel::DEBUG, DeviceType::CAMERA);
        isLoopSolveImage = false;
        Logger::Log("stopLoopSolveImage finish!", LogLevel::DEBUG, DeviceType::CAMERA);
    }

    else if (message == "EndCaptureAndSolve")
    {
        Logger::Log("EndCaptureAndSolve ...", LogLevel::DEBUG, DeviceType::CAMERA);
        EndCaptureAndSolve = true;
        Logger::Log("EndCaptureAndSolve finish!", LogLevel::DEBUG, DeviceType::CAMERA);
    }

    else if (message == "getStagingSolveResult")
    {
        Logger::Log("getStagingSolveResult ...", LogLevel::DEBUG, DeviceType::CAMERA);
        RecoverySloveResul();
        Logger::Log("getStagingSolveResult finish!", LogLevel::DEBUG, DeviceType::CAMERA);
    }

    else if (message == "ClearSloveResultList")
    {
        Logger::Log("ClearSloveResultList ...", LogLevel::DEBUG, DeviceType::CAMERA);
        ClearSloveResultList();
        Logger::Log("ClearSloveResultList finish!", LogLevel::DEBUG, DeviceType::CAMERA);
    }

    else if (message == "getOriginalImage")
    {
        Logger::Log("getOriginalImage ...", LogLevel::DEBUG, DeviceType::CAMERA);
        saveFitsAsPNG(QString::fromStdString("/dev/shm/ccd_simulator_original.fits"), false);
        Logger::Log("getOriginalImage finish!", LogLevel::DEBUG, DeviceType::CAMERA);
    }

    else if (message == "getGPIOsStatus")
    {
        Logger::Log("getGPIOsStatus ...", LogLevel::DEBUG, DeviceType::MAIN);
        getGPIOsStatus();
        Logger::Log("getGPIOsStatus finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "SwitchOutPutPower")
    {
        Logger::Log("SwitchOutPutPower ...", LogLevel::DEBUG, DeviceType::MAIN);
        int index = parts[1].trimmed().toInt();
        int value;

        if (index == 1)
        {
            value = readGPIOValue(GPIO_PIN_1);
            if (value == 1)
            {
                setGPIOValue(GPIO_PIN_1, "0");
                value = readGPIOValue(GPIO_PIN_1);
            }
            else
            {
                setGPIOValue(GPIO_PIN_1, "1");
                value = readGPIOValue(GPIO_PIN_1);
            }
        }
        else if (index == 2)
        {
            value = readGPIOValue(GPIO_PIN_2);
            if (value == 1)
            {
                setGPIOValue(GPIO_PIN_2, "0");
                value = readGPIOValue(GPIO_PIN_2);
            }
            else
            {
                setGPIOValue(GPIO_PIN_2, "1");
                value = readGPIOValue(GPIO_PIN_2);
            }
        }

        emit wsThread->sendMessageToClient("OutputPowerStatus:" + QString::number(index) + ":" + QString::number(value));
        Logger::Log("SwitchOutPutPower finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "SetBinning")
    {
        glMainCameraBinning = parts[1].trimmed().toInt();
        Logger::Log("Set Binning to " + std::to_string(glMainCameraBinning), LogLevel::DEBUG, DeviceType::MAIN);
        Tools::saveParameter("MainCamera", "Binning", parts[1].trimmed());

        // if(dpMainCamera != NULL) {
        //     indi_Client->setCCDBinnign(dpMainCamera, 2, 2);
        // }
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "SetCameraTemperature")
    {
        CameraTemperature = parts[1].trimmed().toDouble();
        Logger::Log("Set Camera Temperature to " + std::to_string(CameraTemperature), LogLevel::DEBUG, DeviceType::MAIN);
        Tools::saveParameter("MainCamera", "Temperature", parts[1].trimmed());
        if (dpMainCamera != NULL)
        {
            indi_Client->setTemperature(dpMainCamera, CameraTemperature);
        }
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "SetCameraGain")
    {
        CameraGain = parts[1].trimmed().toDouble();
        Logger::Log("Set Camera Gain to " + std::to_string(CameraGain), LogLevel::DEBUG, DeviceType::MAIN);
        Tools::saveParameter("MainCamera", "Gain", parts[1].trimmed());
        if (dpMainCamera != NULL)
        {
            indi_Client->setCCDGain(dpMainCamera, CameraGain);
        }
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "SetMainCameraAutoSave")
    {
        mainCameraAutoSave = (parts[1].trimmed() == "true" || parts[1].trimmed() == "1");
        Logger::Log("Set MainCamera Auto Save to " + std::string(mainCameraAutoSave ? "true" : "false"), LogLevel::DEBUG, DeviceType::MAIN);
        Tools::saveParameter("MainCamera", "AutoSave", parts[1].trimmed());
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "SetMainCameraSaveFailedParse")
    {
        mainCameraSaveFailedParse = (parts[1].trimmed() == "true" || parts[1].trimmed() == "1");
        Logger::Log("Set MainCamera Save Failed Parse to " + std::string(mainCameraSaveFailedParse ? "true" : "false"), LogLevel::DEBUG, DeviceType::MAIN);
        Tools::saveParameter("MainCamera", "SaveFailedParse", parts[1].trimmed());
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "SetMainCameraSaveFolder")
    {
        QString Mode = parts[1].trimmed();
        if(Mode == "local" || Mode == "default") {  // Compatible with old "default"
            ImageSaveBaseDirectory = QString::fromStdString(ImageSaveBasePath);
            saveMode = "local";
            Logger::Log("Set MainCamera Save Folder to local: " + ImageSaveBaseDirectory.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        } else {
            // Get the path from the mapping table based on the USB disk name
            if (usbMountPointsMap.contains(Mode)) {
                QString usb_mount_point = usbMountPointsMap[Mode];
                QString folderName = "QUARCS_ImageSave";
                ImageSaveBaseDirectory = usb_mount_point + "/" + folderName;
                saveMode = Mode;  // Save USB disk name
                Logger::Log("Set MainCamera Save Folder to USB: " + Mode.toStdString() + " -> " + ImageSaveBaseDirectory.toStdString(), LogLevel::INFO, DeviceType::MAIN);
            } else {
                // The USB disk does not exist, fall back to the default path.
                ImageSaveBaseDirectory = QString::fromStdString(ImageSaveBasePath);
                saveMode = "local";
                Logger::Log("Set MainCamera Save Folder: USB '" + Mode.toStdString() + "' not found, using local", LogLevel::WARNING, DeviceType::MAIN);
                emit wsThread->sendMessageToClient("CaptureImageSaveStatus:USB-NotAvailable");
            }
        }
        // Use "local" instead of "default" when saving
        QString saveValue = (Mode == "default") ? "local" : Mode;
        Tools::saveParameter("MainCamera", "Save Folder", saveValue);
    }


    else if (parts.size() == 5 && parts[0].trimmed() == "GuiderCanvasClick")
    {
        int CanvasWidth = parts[1].trimmed().toInt();
        int CanvasHeight = parts[2].trimmed().toInt();
        int Click_X = parts[3].trimmed().toInt();
        int Click_Y = parts[4].trimmed().toInt();

        Logger::Log("GuiderCanvasClick:" + std::to_string(CanvasWidth) + "," + std::to_string(CanvasHeight) + "," + std::to_string(Click_X) + "," + std::to_string(Click_Y), LogLevel::DEBUG, DeviceType::MAIN);

        if (glPHD_CurrentImageSizeX != 0 && glPHD_CurrentImageSizeY != 0)
        {
            Logger::Log("PHD2ImageSize:" + std::to_string(glPHD_CurrentImageSizeX) + "," + std::to_string(glPHD_CurrentImageSizeY), LogLevel::DEBUG, DeviceType::MAIN);
            double ratioZoomX = (double)glPHD_CurrentImageSizeX / CanvasWidth;
            double ratioZoomY = (double)glPHD_CurrentImageSizeY / CanvasHeight;
            Logger::Log("ratioZoom:" + std::to_string(ratioZoomX) + "," + std::to_string(ratioZoomY), LogLevel::DEBUG, DeviceType::MAIN);
            double PHD2Click_X = (double)Click_X * ratioZoomX*glPHD_ImageScale;
            double PHD2Click_Y = (double)Click_Y * ratioZoomY*glPHD_ImageScale;
            Logger::Log("PHD2Click:" + std::to_string(PHD2Click_X) + "," + std::to_string(PHD2Click_Y), LogLevel::DEBUG, DeviceType::MAIN);
            call_phd_StarClick(PHD2Click_X, PHD2Click_Y);
        }
    }

    else if (message == "getQTClientVersion")
    {
        Logger::Log("getQTClientVersion ...", LogLevel::DEBUG, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("QTClientVersion:" + QString::fromStdString(QT_Client_Version));
    }

    // Get the total version number (from the environment variable QUARCS_TOTAL_VERSION, format x.x.x)
    else if (message == "getTotalVersion")
    {
        Logger::Log("getTotalVersion ...", LogLevel::DEBUG, DeviceType::MAIN);

        // Read the total version number from the environment variable, falling back to 0.0.0 if not set
        QByteArray envVersion = qgetenv("QUARCS_TOTAL_VERSION");
        QString totalVersion = envVersion.isEmpty() ? "0.0.0" : QString::fromUtf8(envVersion);

        emit wsThread->sendMessageToClient("TotalVersion:" + totalVersion);
    }

    else if (message == "getHotspotName")
    {
        QString HostpotName = getHotspotName();
        Logger::Log("HotspotName:" + HostpotName.toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("HotspotName:" + HostpotName);
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "editHotspotName")
    {
        Logger::Log("editHotspotName ...", LogLevel::DEBUG, DeviceType::MAIN);
        QString HostpotName = parts[1].trimmed();
        editHotspotName(HostpotName);
        Logger::Log("editHotspotName finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (message == "restartHotspot10s")
    {
        Logger::Log("restartHotspot10s ...", LogLevel::DEBUG, DeviceType::MAIN);
        restartHotspotWithDelay(10);
        Logger::Log("restartHotspot10s finish (command issued)", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (parts.size() == 4 && parts[0].trimmed() == "DSLRCameraInfo")
    {
        Logger::Log("DSLRCameraInfo ...", LogLevel::DEBUG, DeviceType::MAIN);
        int Width = parts[1].trimmed().toInt();
        int Height = parts[2].trimmed().toInt();
        double PixelSize = parts[3].trimmed().toDouble();

        if (dpMainCamera != NULL)
        {
            DSLRsInfo DSLRsInfo;
            DSLRsInfo.Name = dpMainCamera->getDeviceName();
            DSLRsInfo.SizeX = Width;
            DSLRsInfo.SizeY = Height;
            DSLRsInfo.PixelSize = PixelSize;
            Tools::saveDSLRsInfo(DSLRsInfo);
            NotSetDSLRsInfo = false;
            // indi_Client->setCCDBasicInfo(dpMainCamera, Width, Height, PixelSize, PixelSize, PixelSize, 8);
            AfterDeviceConnect(dpMainCamera);

            
            Logger::Log("DSLRCameraInfo finish!", LogLevel::DEBUG, DeviceType::MAIN);
        }
        else
        {
            Logger::Log("DSLRCameraInfo failed! Main Camera is NULL", LogLevel::DEBUG, DeviceType::MAIN);
        }
    }
    else if (parts.size() == 3 && parts[0].trimmed() == "saveToConfigFile")
    {
        Logger::Log("saveToConfigFile ...", LogLevel::DEBUG, DeviceType::MAIN);
        QString ConfigName = parts[1].trimmed();
        QString ConfigValue = parts[2].trimmed();

        setClientSettings(ConfigName, ConfigValue);
        if (ConfigName == "Coordinates")
        {
            QStringList parts = ConfigValue.split(",");
            if (parts.size() >= 2)
            {
                setMountLocation(parts[0], parts[1]);
            }
        }
        if (ConfigName == "FocalLength")
        {
            glFocalLength = ConfigValue.toInt();
        }

        Logger::Log("saveToConfigFile finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "GuiderFocalLength")
    {
        int FocalLength = parts[1].trimmed().toInt();
        Logger::Log("Set Guider Focal Length to " + std::to_string(FocalLength), LogLevel::DEBUG, DeviceType::MAIN);

        call_phd_FocalLength(FocalLength);
    }

    else if (message == "RestartRaspberryPi")
    {
        Logger::Log("RestartRaspberryPi ...", LogLevel::DEBUG, DeviceType::MAIN);
        system("reboot");
        Logger::Log("RestartRaspberryPi finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (message == "ShutdownRaspberryPi")
    {
        Logger::Log("ShutdownRaspberryPi ...", LogLevel::DEBUG, DeviceType::MAIN);
        system("shutdown -h now");
        Logger::Log("ShutdownRaspberryPi finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "MultiStarGuider")
    {
        bool isMultiStar = (parts[1].trimmed() == "true");
        Logger::Log("Set Multi Star Guider to" + std::to_string(isMultiStar), LogLevel::DEBUG, DeviceType::MAIN);

        call_phd_MultiStarGuider(isMultiStar);
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "GuiderPixelSize")
    {
        double PixelSize = parts[1].trimmed().toDouble();
        Logger::Log("Set Guider Pixel Size to" + std::to_string(PixelSize), LogLevel::DEBUG, DeviceType::MAIN);

        call_phd_CameraPixelSize(PixelSize);
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "GuiderGain")
    {
        int Gain = parts[1].trimmed().toInt();
        Logger::Log("Set Guider Gain to" + std::to_string(Gain), LogLevel::DEBUG, DeviceType::MAIN);

        call_phd_CameraGain(Gain);
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "CalibrationDuration")
    {
        int StepSize = parts[1].trimmed().toInt();
        Logger::Log("Set Calibration Duration to" + std::to_string(StepSize), LogLevel::DEBUG, DeviceType::MAIN);

        call_phd_CalibrationDuration(StepSize);
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "RaAggression")
    {
        int Aggression = parts[1].trimmed().toInt();
        Logger::Log("Set Ra Aggression to" + std::to_string(Aggression), LogLevel::DEBUG, DeviceType::MAIN);

        call_phd_RaAggression(Aggression);
    }

    else if (parts.size() == 2 && parts[0].trimmed() == "DecAggression")
    {
        int Aggression = parts[1].trimmed().toInt();
        Logger::Log("Set Dec Aggression to" + std::to_string(Aggression), LogLevel::DEBUG, DeviceType::MAIN);

        call_phd_DecAggression(Aggression);
    }
    else if (parts.size() == 3 && parts[0].trimmed() == "ConnectDriver")
    {
        QString DriverName = parts[1].trimmed();
        QString DriverType = parts[2].trimmed();
        Logger::Log("Connect Driver to " + DriverName.toStdString() + " with type " + DriverType.toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        ConnectDriver(DriverName, DriverType);
    }
    else if (parts.size() == 3 && parts[0].trimmed() == "DisconnectDevice")
    {
        QString DeviceName = parts[1].trimmed();
        QString DeviceType = parts[2].trimmed();
        Logger::Log("Disconnect " + DeviceType.toStdString() + " Device(" + DeviceName.toStdString() + ")", LogLevel::DEBUG, DeviceType::MAIN);
        DisconnectDevice(indi_Client, DeviceName, DeviceType);
    }
    else if (message == "loadSelectedDriverList")
    {
        Logger::Log("loadSelectedDriverList ...", LogLevel::DEBUG, DeviceType::MAIN);
        loadSelectedDriverList();
        Logger::Log("loadSelectedDriverList finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (message == "loadBindDeviceList")
    {
        Logger::Log("loadBindDeviceList ...", LogLevel::DEBUG, DeviceType::MAIN);
        loadBindDeviceList(indi_Client);
        Logger::Log("loadBindDeviceList finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (message == "loadBindDeviceTypeList")
    {
        Logger::Log("loadBindDeviceTypeList ...", LogLevel::DEBUG, DeviceType::MAIN);
        loadBindDeviceTypeList();
        Logger::Log("loadBindDeviceTypeList finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "disconnectSelectDriver")
    {
        Logger::Log("disconnect Driver " + parts[1].trimmed().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        disconnectDriver(parts[1]);
        Logger::Log("disconnect Driver " + parts[1].trimmed().toStdString() + " success!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "FocuserMoveState")
    {
        Logger::Log("FocuserMoveState ...", LogLevel::DEBUG, DeviceType::FOCUSER);
        if (parts[1].trimmed() == "true")
        {
            focusMoveEndTime = 2;
        }
        Logger::Log("FocuserMoveState finish!", LogLevel::DEBUG, DeviceType::FOCUSER);
    }
    else if (parts.size() == 2 && parts[0].trimmed() == "FocusLoopShooting")
    {
        Logger::Log("FocusLoopShooting ...", LogLevel::DEBUG, DeviceType::FOCUSER);
        focusLoopShooting(parts[1].trimmed() == "true");
        Logger::Log("FocusLoopShooting finish!", LogLevel::DEBUG, DeviceType::FOCUSER);
    }
    else if (parts[0].trimmed() == "getFocuserLoopingState")
    {
        Logger::Log("getFocuserLoopingState ...", LogLevel::DEBUG, DeviceType::FOCUSER);
        getFocuserLoopingState();
        Logger::Log("getFocuserLoopingState finish!", LogLevel::DEBUG, DeviceType::FOCUSER);
    }
    else if (parts[0].trimmed() == "getROIInfo")
    {
        Logger::Log("getRedBoxState ...", LogLevel::DEBUG, DeviceType::MAIN);
        sendRoiInfo();
        Logger::Log("getRedBoxState finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (parts[0].trimmed() == "sendRedBoxState" && parts.size() == 4)
    {
        Logger::Log("sendRedBoxState ...", LogLevel::DEBUG, DeviceType::MAIN);
        if (parts[1].trimmed().toInt() == BoxSideLength && parts[2].trimmed().toDouble() == roiAndFocuserInfo["ROI_x"] && parts[3].trimmed().toDouble() == roiAndFocuserInfo["ROI_y"])
        {
            return;
        }
        BoxSideLength = parts[1].trimmed().toInt();
        roiAndFocuserInfo["BoxSideLength"] = BoxSideLength;
        roiAndFocuserInfo["ROI_x"] = parts[2].trimmed().toDouble();
        roiAndFocuserInfo["ROI_y"] = parts[3].trimmed().toDouble();
        Tools::saveParameter("MainCamera", "ROI_x", parts[2].trimmed());
        Tools::saveParameter("MainCamera", "ROI_y", parts[3].trimmed());
        Logger::Log("sendRedBoxState finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (parts[0].trimmed() == "sendVisibleArea" && parts.size() == 4)
    {
        Logger::Log("sendVisibleArea ...", LogLevel::DEBUG, DeviceType::MAIN);
        roiAndFocuserInfo["VisibleX"] = parts[1].trimmed().toDouble();
        roiAndFocuserInfo["VisibleY"] = parts[2].trimmed().toDouble();
        roiAndFocuserInfo["scale"] = parts[3].trimmed().toDouble();
    }
    else if (parts[0].trimmed() == "sendSelectStars" && parts.size() == 3)
    {
        Logger::Log("sendSelectStars ...", LogLevel::DEBUG, DeviceType::MAIN);
        roiAndFocuserInfo["SelectStarX"] = parts[1].trimmed().toDouble();
        roiAndFocuserInfo["SelectStarY"] = parts[2].trimmed().toDouble();
        // NewSelectStar = true;
        Logger::Log("sendSelectStars finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (parts[0].trimmed() == "testQtServerProcess")
    {
        // Logger::Log("testQtServerProcess ... .....................", LogLevel::DEBUG, DeviceType::MAIN);
        emit wsThread->sendProcessCommandReturn("ServerInitSuccess");
    }
    else if (parts[0].trimmed() == "getMainCameraParameters")
    {
        Logger::Log("getMainCameraParameters ...", LogLevel::DEBUG, DeviceType::MAIN);
        getMainCameraParameters();
        Logger::Log("getMainCameraParameters finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (parts[0].trimmed() == "getMountParameters")
    {
        Logger::Log("getMountParameters ...", LogLevel::DEBUG, DeviceType::MAIN);
        getMountParameters();
        Logger::Log("getMountParameters finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (parts[0].trimmed() == "SynchronizeTime")
    {
        QRegularExpression rx("SynchronizeTime:(\\d{2}:\\d{2}:\\d{2}):(\\d{4}-\\d{2}-\\d{2})");
        QRegularExpressionMatch match = rx.match(message);
        //        int pos = rx.indexIn(message);
        if (match.hasMatch())
        {
            QString time = match.captured(1);
            QString date = match.captured(2);
            Logger::Log("SynchronizeTime ...", LogLevel::DEBUG, DeviceType::MAIN);
            synchronizeTime(time, date);
            setMountUTC(time, date);
            Logger::Log("SynchronizeTime finish!", LogLevel::DEBUG, DeviceType::MAIN);
        }
    }
    else if (parts[0].trimmed() == "localMessage")
    {
        if (parts.size() >= 4)
        {
            Logger::Log("localMessage ...", LogLevel::DEBUG, DeviceType::MAIN);
            localLat = parts[1].trimmed();
            localLon = parts[2].trimmed();
            if (parts.size() >= 5)
            {
                localAppVersion = parts[4].trimmed();
            }
            // Logger::Log("1-----------Initial parameter settings: " + localLat.toStdString() + "," + localLon.toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
            if (parts[3].trimmed() == "zh")
            {
                setClientSettings("ClientLanguage", "cn");
                localLanguage = "cn";
            }
            else
            {
                setClientSettings("ClientLanguage", parts[3].trimmed());
                localLanguage = parts[3].trimmed();
            }
            emit wsThread->sendMessageToClient(message);
            Logger::Log("localMessage finish!", LogLevel::DEBUG, DeviceType::MAIN);
        }
        else
        {
            // Logger::Log("1------------Restore browser status...", LogLevel::DEBUG, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("localMessage:" + localLat + ":" + localLon + ":" + localLanguage);
        }
    }
    else if (parts[0].trimmed() == "currectLocation" && parts.size() == 3)
    {
        Logger::Log("currectLocation ...", LogLevel::DEBUG, DeviceType::MAIN);
        QString CurrentLocationLat = parts[1].trimmed();
        QString CurrentLocationLon = parts[2].trimmed();
        setMountLocation(CurrentLocationLat, CurrentLocationLon);
        Logger::Log("currectLocation finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (parts[0].trimmed() == "reGetLocation")
    {
        Logger::Log("reGetLocation ...", LogLevel::DEBUG, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("sendGetLocation:" + localLat + ":" + localLon);
        Logger::Log("reGetLocation finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (parts[0].trimmed() == "showRoiImageSuccess" && parts.size() == 2)
    {
        Logger::Log("showRoiImageSuccess ...", LogLevel::DEBUG, DeviceType::MAIN);
        // if (parts[1].trimmed() == "true") focusLoopShooting(true);
        // else focusLoopShooting(false);
        Logger::Log("showRoiImageSuccess finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else if (parts[0].trimmed() == "getLastSelectDevice")
    {
        Logger::Log("getLastSelectDevice ...", LogLevel::DEBUG, DeviceType::MAIN);
        getLastSelectDevice();
    }
    else if (parts[0].trimmed() == "StartAutoPolarAlignment")
    {
        Logger::Log("StartAutoPolarAlignment ...", LogLevel::DEBUG, DeviceType::MAIN);
        if (polarAlignment != nullptr)
        {
            polarAlignment->stopPolarAlignment();
            delete polarAlignment;
            polarAlignment = nullptr;
            Logger::Log("ResetAutoPolarAlignment: Reset successfully", LogLevel::INFO, DeviceType::MAIN);
        }
        else
        {
            Logger::Log("ResetAutoPolarAlignment: polarAlignment is nullptr", LogLevel::WARNING, DeviceType::MAIN);
        }

        bool isSuccess = initPolarAlignment();
        if (isSuccess)
        {
            // Before starting automatic polar alignment, turn off equatorial mount tracking.
            bool trackingDisabled = false;
            if (indi_Client != nullptr && dpMount != nullptr)
            {
                indi_Client->setTelescopeTrackEnable(dpMount, false);
                emit wsThread->sendMessageToClient("TelescopeTrack:OFF");
                Logger::Log("StartAutoPolarAlignment: Telescope tracking disabled for polar alignment",
                            LogLevel::INFO, DeviceType::MAIN);
                trackingDisabled = true;
            }

            if (polarAlignment->startPolarAlignment())
            {
                Logger::Log("StartAutoPolarAlignment: Started successfully", LogLevel::INFO, DeviceType::MAIN);
            }
            else
            {
                Logger::Log("StartAutoPolarAlignment: Failed to start polar alignment", LogLevel::ERROR, DeviceType::MAIN);
                emit wsThread->sendMessageToClient("StartAutoPolarAlignmentStatus:false:Failed to start polar alignment");

                // Restore previous tracking state when startup fails
                if (trackingDisabled && indi_Client != nullptr && dpMount != nullptr)
                {
                    indi_Client->setTelescopeTrackEnable(dpMount, true);

                    bool isTrack = false;
                    indi_Client->getTelescopeTrackEnable(dpMount, isTrack);
                    emit wsThread->sendMessageToClient(isTrack ? "TelescopeTrack:ON"
                                                               : "TelescopeTrack:OFF");
                    Logger::Log("StartAutoPolarAlignment: Restore telescope tracking because start failed",
                                LogLevel::INFO, DeviceType::MAIN);
                }
            }
        }
        else
        {
            Logger::Log("StartAutoPolarAlignment: Failed to initialize polar alignment", LogLevel::ERROR, DeviceType::MAIN);
        }
    }
    else if (parts[0].trimmed() == "StopAutoPolarAlignment")
    {
        Logger::Log("StopAutoPolarAlignment ...", LogLevel::DEBUG, DeviceType::MAIN);
        if (polarAlignment != nullptr)
        {
            polarAlignment->stopPolarAlignment();
            Logger::Log("StopAutoPolarAlignment: Stopped successfully", LogLevel::INFO, DeviceType::MAIN);
        }
        else
        {
            Logger::Log("StopAutoPolarAlignment: polarAlignment is nullptr", LogLevel::WARNING, DeviceType::MAIN);
        }
        Logger::Log("StopAutoPolarAlignment finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }

    else if (parts[0].trimmed() == "focusMoveToMin")
    {
        Logger::Log("focusMoveToMin ...", LogLevel::DEBUG, DeviceType::FOCUSER);
        focusMoveToMin();
        Logger::Log("focusMoveToMin finish!", LogLevel::DEBUG, DeviceType::FOCUSER);
    }
    else if (parts[0].trimmed() == "focusMoveToMax")
    {
        Logger::Log("focusMoveToMax ...", LogLevel::DEBUG, DeviceType::FOCUSER);
        focusMoveToMax();
        Logger::Log("focusMoveToMax finish!", LogLevel::DEBUG, DeviceType::FOCUSER);
    }
    else if (parts[0].trimmed() == "focusSetTravelRange")
    {
        Logger::Log("focusSetTravelRange ...", LogLevel::DEBUG, DeviceType::FOCUSER);
        focusSetTravelRange();
        Logger::Log("focusSetTravelRange finish!", LogLevel::DEBUG, DeviceType::FOCUSER);
    }
    else if (parts[0].trimmed() == "getFocuserParameters")
    {
        Logger::Log("getFocuserParameters ...", LogLevel::DEBUG, DeviceType::FOCUSER);
        getFocuserParameters();
        Logger::Log("getFocuserParameters finish!", LogLevel::DEBUG, DeviceType::FOCUSER);
    }
    else if (parts[0].trimmed() == "getPolarAlignmentState")
    {
        Logger::Log("getPolarAlignmentState ...", LogLevel::DEBUG, DeviceType::MAIN);
        if (polarAlignment != nullptr)
        {
            if (polarAlignment->isRunning())
            {
                // 1. Get the current status
                PolarAlignmentState currentState = polarAlignment->getCurrentState();
                // 2. Get current information
                QString currentStatusMessage = polarAlignment->getCurrentStatusMessage();
                // 3. Get the current progress
                int progressPercentage = polarAlignment->getProgressPercentage();
                emit wsThread->sendMessageToClient(QString("PolarAlignmentState:") +
                                                        (polarAlignment->isRunning() ? "true" : "false") + ":" +
                                                        QString::number(static_cast<int>(currentState)) + ":" +
                                                        currentStatusMessage + ":" +
                                                        QString::number(progressPercentage));

                // 4. Obtain all currently controllable data
                polarAlignment->sendValidAdjustmentGuideData();
            }
        }else{
            emit wsThread->sendMessageToClient(QString("PolarAlignmentState:false:未启动:未启动:0"));
        }
    }else if (parts[0].trimmed() == "CheckBoxSpace")
    {
        Logger::Log("CheckBoxSpace ...", LogLevel::DEBUG, DeviceType::MAIN);
        getCheckBoxSpace();
        Logger::Log("CheckBoxSpace finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }else if (parts[0].trimmed() == "ClearLogs")
    {
        Logger::Log("ClearLogs ...", LogLevel::DEBUG, DeviceType::MAIN);
        clearLogs();
        Logger::Log("ClearLogs finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }else if (parts[0].trimmed() == "ClearBoxCache")
    {
        Logger::Log("ClearBoxCache ...", LogLevel::DEBUG, DeviceType::MAIN);
        bool clearCache = true;
        bool clearUpdatePack = false;
        bool clearBackup = false;
        // Supports extended protocols in the form ClearBoxCache:1:0:1
        if (parts.size() >= 4)
        {
            clearCache      = (parts[1].trimmed() != "0");
            clearUpdatePack = (parts[2].trimmed() != "0");
            clearBackup     = (parts[3].trimmed() != "0");
        }
        clearBoxCache(clearCache, clearUpdatePack, clearBackup);
        Logger::Log("ClearBoxCache finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }else if (parts[0].trimmed() == "loadSDKVersionAndUSBSerialPath")
    {
        Logger::Log("loadSDKVersionAndUSBSerialPath ...", LogLevel::DEBUG, DeviceType::MAIN);
        loadSDKVersionAndUSBSerialPath();
        Logger::Log("loadSDKVersionAndUSBSerialPath finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }else if (parts[0].trimmed() == "getFocuserState")
    {
        Logger::Log("getFocuserState ...", LogLevel::DEBUG, DeviceType::FOCUSER);
        getFocuserState();

        Logger::Log("getFocuserState finish!", LogLevel::DEBUG, DeviceType::FOCUSER);
    }
    else if (parts[0].trimmed() == "SolveCurrentPosition")
    {
        Logger::Log("SolveCurrentPosition ...", LogLevel::DEBUG, DeviceType::FOCUSER);
        solveCurrentPosition();
        Logger::Log("SolveCurrentPosition finish!", LogLevel::DEBUG, DeviceType::FOCUSER);
    }
    else
    {
        Logger::Log("Unknown message: " + message.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
    }
}

void MainWindow::initINDIServer()
{
    Logger::Log("initINDIServer ...", LogLevel::INFO, DeviceType::MAIN);
    system("pkill indiserver");
    system("rm -f /tmp/myFIFO");
    system("mkfifo /tmp/myFIFO");
    if (!glIndiServer) {
        glIndiServer = new QProcess(this);
    }
    // glIndiServer->setReadChannel(QProcess::StandardOutput);

    // Connect signal to slot function
    // connect(glIndiServer, &QProcess::readyReadStandardOutput, this, &MainWindow::handleIndiServerOutput);
    // connect(glIndiServer, &QProcess::readyReadStandardError, this, &MainWindow::handleIndiServerError);
    QString program = "indiserver";
    QStringList arguments;
    arguments << "-f" << "/tmp/myFIFO" << "-v" << "-p" << "7624";
    glIndiServer->start(program, arguments);
    if (!glIndiServer->waitForStarted(2000)) {
        Logger::Log("Device MAIN: Failed to start indiserver! Error: " +
        glIndiServer->errorString().toStdString(), LogLevel::ERROR, DeviceType::MAIN);
    } else {
        Logger::Log("Device MAIN: initINDIServer finish! (Process PID: " +
        std::to_string(glIndiServer->processId()) + ")", LogLevel::INFO, DeviceType::MAIN);
    }

    // 4. Important: Capture runtime errors
    connect(glIndiServer, &QProcess::readyReadStandardError, this, [this](){
        QByteArray errors = glIndiServer->readAllStandardError();
        Logger::Log("INDI STDERR: " + errors.toStdString(), LogLevel::ERROR, DeviceType::MAIN);
    });
}

// void MainWindow::initINDIServer()
// {
//     system("pkill indiserver");
//     system("rm -f /tmp/myFIFO");
//     system("mkfifo /tmp/myFIFO");
//     glIndiServer = new QProcess();
//     glIndiServer->setReadChannel(QProcess::StandardOutput);
//     glIndiServer->start("indiserver -f /tmp/myFIFO -v -p 7624");
// }

// Slot function: processing standard output
void MainWindow::handleIndiServerOutput()
{
    QByteArray output = glIndiServer->readAllStandardOutput();
    Logger::Log("INDI Server Output: " + output.toStdString(), LogLevel::INFO, DeviceType::MAIN);
}

// Slot function: handling standard error
void MainWindow::handleIndiServerError()
{
    QByteArray error = glIndiServer->readAllStandardError();
    Logger::Log("INDI Server Error: " + error.toStdString(), LogLevel::ERROR, DeviceType::MAIN);
}
int i = 1;
void MainWindow::initINDIClient()
{
    Logger::Log("initINDIClient ...", LogLevel::INFO, DeviceType::MAIN);
    indi_Client = new MyClient();
    indi_Client->setServer("localhost", 7624);
    indi_Client->setConnectionTimeout(3, 0);
    Logger::Log("setConnectionTimeout is 3 seconds!", LogLevel::INFO, DeviceType::MAIN);
    indi_Client->setImageReceivedCallback(
        [this](const std::string &filename, const std::string &devname)
        {
            // Exposure completed
            if (dpMainCamera != NULL)
            {
                if (dpMainCamera->getDeviceName() == devname)
                {
                    glMainCameraStatu = "Displaying";
                    ShootStatus = "Completed";
                    if (autoFocuserIsROI && isAutoFocus)
                    {
                        saveFitsAsJPG(QString::fromStdString(filename), true);
                        Logger::Log("saveFitsAsJPG", LogLevel::DEBUG, DeviceType::MAIN);
                        autoFocus->setCaptureComplete(QString::fromStdString(filename));
                        return;
                    }
                    else if (!autoFocuserIsROI && isAutoFocus)
                    {

                        saveFitsAsPNG(QString::fromStdString(filename), true);

                        Logger::Log("saveFitsAsPNG", LogLevel::DEBUG, DeviceType::MAIN);
                        autoFocus->setCaptureComplete(QString::fromStdString(filename));
                        return;
                    }

                    if (glIsFocusingLooping == false)
                    {
                        emit wsThread->sendMessageToClient("ExposureCompleted");
                        Logger::Log("ExposureCompleted", LogLevel::INFO, DeviceType::CAMERA);
                        if (polarAlignment != nullptr)
                        {
                            if (polarAlignment->isRunning())
                            {
                                polarAlignment->setCaptureEnd(true);
                                Logger::Log("ExposureCompleted, but polarAlignment is not idle", LogLevel::INFO, DeviceType::MAIN);
                                return;
                            }
                        }
                        saveFitsAsPNG(QString::fromStdString(filename), true); // "/dev/shm/ccd_simulator.fits"
                        // saveFitsAsPNG("/home/quarcs/2025_06_26T08_24_13_544.fits", true);
                        // saveFitsAsPNG("/dev/shm/SOLVETEST.fits", true);
                        
                        // Automatically saves images if autosave is on
                        if (mainCameraAutoSave && isScheduleRunning == false)
                        {
                            Logger::Log("Auto Save enabled, saving captured image...", LogLevel::INFO, DeviceType::MAIN);
                            CaptureImageSave();
                        }
                    }
                    else
                    {

                        saveFitsAsJPG(QString::fromStdString(filename), true);

                        Logger::Log("saveFitsAsJPG", LogLevel::DEBUG, DeviceType::MAIN);
                    }
                    // Logger::Log("Photography completed, image saving completed finish!", LogLevel::INFO, DeviceType::MAIN);
                }
            }
        });
    Logger::Log("indi_Client->setImageReceivedCallback finish!", LogLevel::INFO, DeviceType::MAIN);

    indi_Client->setMessageReceivedCallback(
        [this](const std::string &message)
        {
            // qDebug("indi initial message %s", message.c_str());
            QString messageStr = QString::fromStdString(message.c_str());

            // Remove timestamps using regular expressions
            std::regex timestampRegex(R"(\d{4}-\d{2}-\d{2}T\d{2}:\d{2}:\d{2}: )");
            messageStr = QString::fromStdString(std::regex_replace(messageStr.toStdString(), timestampRegex, ""));

            // qDebug("indi extracted information %s", messageStr.toStdString().c_str());
            // Extract and remove log types using regular expressions
            std::regex typeRegex(R"(\[(INFO|WARNING|ERROR)\])");
            std::smatch typeMatch;
            QString logType;
            if (std::regex_search(message, typeMatch, typeRegex) && typeMatch.size() > 1)
            {
                logType = QString::fromStdString(typeMatch[1].str());
                // Remove log type
                messageStr = QString::fromStdString(std::regex_replace(messageStr.toStdString(), typeRegex, ""));
            }

            if (messageStr.contains("Telescope focal length is missing.") ||
                messageStr.contains("Telescope aperture is missing."))
            {
                // Skip printing
                return;
            }
            if (logType == "WARNING")
            {
                DeviceType deviceType = getDeviceTypeFromPartialString(messageStr.toStdString());
                Logger::Log("[INDI SERVER] " + messageStr.toStdString(), LogLevel::WARNING, deviceType);
            }
            else if (logType == "ERROR")
            {
                DeviceType deviceType = getDeviceTypeFromPartialString(messageStr.toStdString());
                Logger::Log("[INDI SERVER] " + messageStr.toStdString(), LogLevel::ERROR, deviceType);
            }
            else
            {
                DeviceType deviceType = getDeviceTypeFromPartialString(messageStr.toStdString());
                Logger::Log("[INDI SERVER] " + messageStr.toStdString(), LogLevel::INFO, deviceType);
            }

            std::regex regexPattern(R"(OnStep slew/syncError:\s*(.*))");
            std::smatch matchResult;

            if (std::regex_search(message, matchResult, regexPattern))
            {
                if (matchResult.size() > 1)
                {
                    QString errorContent = QString::fromStdString(matchResult[1].str());
                    Logger::Log("OnStep Error: " + errorContent.toStdString(), LogLevel::INFO, DeviceType::MAIN);
                    emit wsThread->sendMessageToClient("OnStep Error:" + errorContent);
                    MountGotoError = true;
                }
            }
        });
    Logger::Log("indi_Client->setMessageReceivedCallback finish!", LogLevel::INFO, DeviceType::MAIN);
}

DeviceType MainWindow::getDeviceTypeFromPartialString(const std::string &typeStr)
{
    // Use the find() method to check if a string contains a specific substring
    if (typeStr.find("Exposure done, downloading image") != std::string::npos || typeStr.find("Download complete") != std::string::npos || typeStr.find("Uploading file.") != std::string::npos || typeStr.find("Image saved to") != std::string::npos)
    {
        // Logger::Log("The information type obtained is camera", LogLevel::INFO, DeviceType::CAMERA);
        return DeviceType::CAMERA;
    }
    else if (0)
    {
        // Logger::Log("DeviceType::MOUNT", LogLevel::INFO, DeviceType::MOUNT);
        return DeviceType::MOUNT;
    }
    else
    {
        // Logger::Log("DeviceType::MAIN", LogLevel::INFO, DeviceType::MAIN);
        return DeviceType::MAIN;
    }
}

void MainWindow::initGPIO()
{
    Logger::Log("Initializing GPIO...", LogLevel::INFO, DeviceType::MAIN);
    // Initialize GPIO_PIN_1
    exportGPIO(GPIO_PIN_1);
    setGPIODirection(GPIO_PIN_1, "out");
    Logger::Log("Set direction of GPIO_PIN_1 to output completed!", LogLevel::INFO, DeviceType::MAIN);

    // Initialize GPIO_PIN_2
    exportGPIO(GPIO_PIN_2);
    setGPIODirection(GPIO_PIN_2, "out");
    Logger::Log("Set direction of GPIO_PIN_2 to output completed!", LogLevel::INFO, DeviceType::MAIN);

    // Set GPIO_PIN_1 to high level
    setGPIOValue(GPIO_PIN_1, "1");
    Logger::Log("Set GPIO_PIN_1 level to high completed!", LogLevel::INFO, DeviceType::MAIN);
    // Set GPIO_PIN_2 to high level
    setGPIOValue(GPIO_PIN_2, "1");
    Logger::Log("Set GPIO_PIN_2 level to high completed!", LogLevel::INFO, DeviceType::MAIN);

    Logger::Log("GPIO initialization completed!", LogLevel::INFO, DeviceType::MAIN);
}

void MainWindow::exportGPIO(const char *pin)
{
    int fd;
    char buf[64];

    // Export GPIO pin
    fd = open(GPIO_EXPORT, O_WRONLY);
    if (fd < 0)
    {
        Logger::Log("Failed to open export file for writing", LogLevel::WARNING, DeviceType::MAIN);
        return;
    }
    snprintf(buf, sizeof(buf), "%s", pin);
    if (write(fd, buf, strlen(buf)) != strlen(buf))
    {
        Logger::Log("Failed to write to export file", LogLevel::INFO, DeviceType::MAIN);
        close(fd);
        return;
    }
    close(fd);
    Logger::Log("GPIO pin export successful", LogLevel::INFO, DeviceType::MAIN);
}

void MainWindow::setGPIODirection(const char *pin, const char *direction)
{
    int fd;
    char path[128];

    // Set GPIO direction
    snprintf(path, sizeof(path), GPIO_PATH "/gpio%s/direction", pin);
    fd = open(path, O_WRONLY);
    if (fd < 0)
    {
        Logger::Log("Failed to open GPIO direction file for writing", LogLevel::WARNING, DeviceType::MAIN);
        return;
    }
    if (write(fd, direction, strlen(direction) + 1) != strlen(direction) + 1)
    {
        Logger::Log("Failed to set GPIO direction", LogLevel::INFO, DeviceType::MAIN);
        close(fd);
        return;
    }
    close(fd);
    Logger::Log("GPIO direction set successfully", LogLevel::INFO, DeviceType::MAIN);
}
void MainWindow::setGPIOValue(const char *pin, const char *value)
{
    int fd;
    char path[128];

    // Set GPIO value
    snprintf(path, sizeof(path), GPIO_PATH "/gpio%s/value", pin);
    fd = open(path, O_WRONLY);
    if (fd < 0)
    {
        Logger::Log("Failed to open GPIO value file for writing", LogLevel::WARNING, DeviceType::MAIN);
        return;
    }
    if (write(fd, value, strlen(value) + 1) != strlen(value) + 1)
    {
        Logger::Log("Failed to write to GPIO value", LogLevel::INFO, DeviceType::MAIN);
        close(fd);
        return;
    }
    close(fd);
    Logger::Log("GPIO value set successfully", LogLevel::INFO, DeviceType::MAIN);
}
int MainWindow::readGPIOValue(const char *pin)
{
    int fd;
    char path[128];
    char value[3]; // Store the read value

    // Construct the path to the GPIO value file
    snprintf(path, sizeof(path), GPIO_PATH "/gpio%s/value", pin);

    // Open the GPIO value file for reading
    fd = open(path, O_RDONLY);
    if (fd < 0)
    {
        Logger::Log("Failed to open GPIO value file for reading", LogLevel::WARNING, DeviceType::MAIN);
        return -1; // Return -1 to indicate read failure
    }

    // Read the file content
    if (read(fd, value, sizeof(value)) < 0)
    {
        Logger::Log("Failed to read GPIO value", LogLevel::INFO, DeviceType::MAIN);
        close(fd);
        return -1; // Return -1 to indicate read failure
    }

    close(fd);

    // Determine if the read value is '1' or '0'
    if (value[0] == '1')
    {
        return 1; // Return 1 to indicate high level
    }
    else if (value[0] == '0')
    {
        return 0; // Return 0 to indicate low level
    }
    else
    {
        return -1; // If the value is not '0' or '1', return -1
    }
}
void MainWindow::getGPIOsStatus()
{
    int value1 = readGPIOValue(GPIO_PIN_1);
    emit wsThread->sendMessageToClient("OutputPowerStatus:1:" + QString::number(value1));
    int value2 = readGPIOValue(GPIO_PIN_2);
    emit wsThread->sendMessageToClient("OutputPowerStatus:2:" + QString::number(value2));
}

void MainWindow::onTimeout()
{
    ShowPHDdata();

    // Show equatorial mount pointing
    mountDisplayCounter++;
    if (dpMount != NULL)
    {
        if (dpMount->isConnected())
        {
            if (mountDisplayCounter >= 5)
            {
                double RA_HOURS, DEC_DEGREE;
                indi_Client->getTelescopeRADECJNOW(dpMount, RA_HOURS, DEC_DEGREE);
                double CurrentRA_Degree = Tools::HourToDegree(RA_HOURS);
                double CurrentDEC_Degree = DEC_DEGREE;

                emit wsThread->sendMessageToClient("TelescopeRADEC:" 
                    + QString::number(CurrentRA_Degree) 
                    + ":" + QString::number(CurrentDEC_Degree));

                // Logger::Log("Currently pointing to:RA:" + std::to_string(RA_HOURS) + "Hours,DEC:" + std::to_string(CurrentDEC_Degree) + "Degree", LogLevel::INFO, DeviceType::MAIN);

                // Directly execute the original "slow" query content each time
                bool isParked = false;
                indi_Client->getTelescopePark(dpMount, isParked);
                emit wsThread->sendMessageToClient(
                    isParked ? "TelescopePark:ON" : "TelescopePark:OFF");
                
                QString NewTelescopePierSide;
                indi_Client->getTelescopePierSide(dpMount, NewTelescopePierSide);
                if (NewTelescopePierSide != TelescopePierSide)
                {
                    // If there is a side change in direction, it means that a mid-heaven flip has occurred. Determine whether the flip is completed.
                    if (indi_Client->mountState.isMovingNow() == false) {
                        emit wsThread->sendMessageToClient("FlipStatus:success");
                        TelescopePierSide = NewTelescopePierSide;
                    }
                }
                emit wsThread->sendMessageToClient("TelescopePierSide:" + NewTelescopePierSide);

                indi_Client->getTelescopeMoving(dpMount);

                bool isTrack = false;
                indi_Client->getTelescopeTrackEnable(dpMount, isTrack);
                emit wsThread->sendMessageToClient(isTrack ? "TelescopeTrack:ON" 
                                                           : "TelescopeTrack:OFF");

                if (!FirstRecordTelescopePierSide)
                {
                    if (FirstTelescopePierSide != TelescopePierSide)
                        isMeridianFlipped = true;
                    else
                        isMeridianFlipped = false;
                }

                emit wsThread->sendMessageToClient("TelescopeStatus:" + TelescopeControl_Status());

                mountDisplayCounter = 0;

                // const MeridianStatus ms = checkMeridianStatus(); //This calculates the current time from midday
                // switch (ms.event) {
                //   case FlipEvent::Started: emit wsThread->sendMessageToClient("MeridianFlip:STARTED"); break;
                //   case FlipEvent::Done: emit wsThread->sendMessageToClient("MeridianFlip:DONE"); break;
                //   case FlipEvent::Failed: emit wsThread->sendMessageToClient("MeridianFlip:FAILED"); break;
                //   default: break;
                // }

                // if (!std::isnan(ms.etaMinutes)) {
                //     // Display rules: bound to flipping requirements -negative signs need to be flipped and displayed, but positive signs do not need to be displayed
                //     const bool showNeg = ms.needsFlip;
                //     const double absMinutes = std::fabs(ms.etaMinutes);
                //     const int totalSeconds = static_cast<int>(std::llround(absMinutes *60.0));
                //     const int hours = totalSeconds /3600;
                //     const int mins = (totalSeconds % 3600) /60;
                //     const int secs = totalSeconds % 60;

                //     const QString hms = QString("%1%2:%3:%4")
                //                             .arg(showNeg ? "-" : "")
                //                             .arg(hours, 2, 10, QLatin1Char('0'))
                //                             .arg(mins,  2, 10, QLatin1Char('0'))
                //                             .arg(secs,  2, 10, QLatin1Char('0'));
                //     emit wsThread->sendMessageToClient("MeridianETA_hms:" + hms);
                //     Logger::Log("MeridianETA_hms:" + hms.toStdString() + " side:" + TelescopePierSide.toStdString() + " needflip:" + (ms.needsFlip ? "true" : "false"), LogLevel::INFO, DeviceType::MAIN);
                // }

                //TODO: There is a problem with the current judgment method and the judgment needs to be revised.
                // Add judgment. When it is necessary to perform automatic mid-heaven flip at this time, and the device is set to automatic mid-heaven flip, then perform automatic mid-heaven flip.
                // if (ms.needsFlip && isAutoFlip && indi_Client->mountState.isFlipping == false && indi_Client->mountState.isFlipBacking == false) {
                //     // Ready to flip
                //     if (flipPrepareTime >= 0) {
                //         flipPrepareTime-=2;
                //         emit wsThread->sendMessageToClient("FlipStatus:FlipPrepareTime," + QString::number(flipPrepareTime));
                //     }
                //     else {
                //         emit wsThread->sendMessageToClient("FlipStatus:start");
                //         indi_Client->startFlip(dpMount);
                //     }
                // }else{
                //     flipPrepareTime = flipPrepareTimeDefault;
                // }
                // if (indi_Client->mountState.isFlipping == true || indi_Client->mountState.isFlipBacking == true) {
                //     emit wsThread->sendMessageToClient("FlipStatus:start");
                // }

            }
        }
    }

    MainCameraStatusCounter++;
    if (dpMainCamera != NULL)
    {
        if (MainCameraStatusCounter >= 5)
        {
            emit wsThread->sendMessageToClient("MainCameraStatus:" + glMainCameraStatu);
            MainCameraStatusCounter = 0;
            double CameraTemp;
            uint32_t ret;
            ret = indi_Client->getTemperature(dpMainCamera, CameraTemp);
            if (ret == QHYCCD_SUCCESS)
            {
                emit wsThread->sendMessageToClient("MainCameraTemperature:" + QString::number(CameraTemp));
            }

        }
    }
}

MeridianStatus MainWindow::checkMeridianStatus()
{
    MeridianStatus out;

    if (!dpMount || !dpMount->isConnected())
        return out;

    // Read PierSide (the direction side reported by the device)
    QString pier = "UNKNOWN";
    indi_Client->getTelescopePierSide(dpMount, pier); // "EAST"/"WEST"/"UNKNOWN"

    // --------Midday ETA (minutes) --------
    // 1) Current right ascension (hour)
    double raH = 0.0, decDeg = 0.0;
    indi_Client->getTelescopeRADECJNOW(dpMount, raH, decDeg);

    // 2) LST hours (TIME_LST takes precedence; otherwise UTC+longitude estimate)
    auto norm24 = [](double h){ h=fmod(h,24.0); if(h<0) h+=24.0; return h; };
    // Extrapolate and unify possible units such as degrees/second to hours, and then normalize to [0,24)
    auto toHours = [&](double v)->double {
        if (std::isnan(v)) return v;
        double x = v;
        // If it is seconds (0..86400), convert to hours
        if (std::fabs(x) > 24.0 && std::fabs(x) <= 86400.0) x /= 3600.0;
//If it is degrees (0..360), convert to hours
        if (std::fabs(x) > 24.0 && std::fabs(x) <= 360.0)  x /= 15.0;
        // If it exceeds one circle (>360 degrees, etc.), first normalize by degrees and then turn to hours.
        if (std::fabs(x) > 360.0) x = fmod(x, 360.0) / 15.0;
        return norm24(x);
    };
    double lstH = std::numeric_limits<double>::quiet_NaN();

    // 2.1 Use INDI::PropertyNumber to read TIME_LST (to avoid p->np errors)
    if (true) {
        INDI::PropertyNumber lst = dpMount->getNumber("TIME_LST");
        if (lst.isValid() && lst.size() > 0) {
            lstH = toHours(lst[0].getValue());   // Unified to hours
        }
    }

    // 2.2 If there is no TIME_LST, get the longitude from GEOGRAPHIC_COORD and use UTC to calculate LST
    if (std::isnan(lstH)) {
        double lonDeg = std::numeric_limits<double>::quiet_NaN();
        INDI::PropertyNumber geo = dpMount->getNumber("GEOGRAPHIC_COORD");
        if (geo.isValid()) {
            // The usual order is LAT(0), LONG(1), ELEV(2); if your driver uses named items, you can also use geo["LONG"].getValue()
            if (geo.size() >= 2)
                lonDeg = geo[1].getValue();
        }
        if (!std::isnan(lonDeg)) {
            const QDateTime utc = QDateTime::currentDateTimeUtc();
            const int Y=utc.date().year(), M=utc.date().month(), D=utc.date().day();
            const int H=utc.time().hour(), Min=utc.time().minute(), S=utc.time().second(), ms=utc.time().msec();

            auto jdUTC = [&](int Y,int M,int D,int H,int Min,int S,int ms)->double{
                int a=(14-M)/12, y=Y+4800-a, m=M+12*a-3;
                long JDN=D+(153*m+2)/5+365*y+y/4-y/100+y/400-32045;
                double dayfrac=(H-12)/24.0 + Min/1440.0 + (S + ms/1000.0)/86400.0;
                return JDN + dayfrac;
            };
            const double JD = jdUTC(Y,M,D,H,Min,S,ms);
            const double Dd = JD - 2451545.0;
            double GMST = 18.697374558 + 24.06570982441908 * Dd; // hour
            lstH = norm24(GMST + lonDeg/15.0);
        }
    }

    // Clean RA units and standardize to hours
    raH = toHours(raH);

    if (!std::isnan(lstH)) {
        // Use the half-open interval [-12, 12) to normalize the angle and avoid boundary jitter
        auto wrap12 = [](double h){ while (h < -12.0) h += 24.0; while (h >= 12.0) h -= 24.0; return h; };
        const double haPrincipal = wrap12(lstH - raH); // Hours; <0 East side, >0 West side
        // Continuous hour angle (prevents sign changes when moving from -12h to +12h at lower culmination)
        static bool hasContHA = false;
        static double contHA = 0.0;
        static double lastHAPrincipal = 0.0;
        if (!hasContHA) {
            contHA = haPrincipal;
            lastHAPrincipal = haPrincipal;
            hasContHA = true;
        } else {
            double delta = haPrincipal - lastHAPrincipal;
            if (delta > 12.0) delta -= 24.0;
            else if (delta < -12.0) delta += 24.0;
            contHA += delta;
            lastHAPrincipal = haPrincipal;
        }
        const bool isPastUpper = (haPrincipal > 0.0);

        // HOME position also participates in flip detection (no special suppression)

        // Based on |HA| = 6h division (Note: the 6h here is hour angle HA, not RA):
        // - Upper meridian half-period |HA| < 6h: HA ≥ 0 → EAST, HA < 0 → WEST
        // - Lower meridian half-period |HA| ≥ 6h: reverse mapping (symmetry relation)
        constexpr double kHalfCycleHAHours = 6.0;
        constexpr double kBoundaryTolH = 0.02; // ≈1.2 minutes tolerance
        const bool isLowerRegion = (std::fabs(haPrincipal) >= (kHalfCycleHAHours - kBoundaryTolH));
        bool eastMapping = (haPrincipal >= 0.0);
        if (isLowerRegion) eastMapping = !eastMapping;
        QString theoreticalPier = eastMapping ? "EAST" : "WEST";
        if (pier == "UNKNOWN") {
            out.needsFlip = false; // Cannot determine or approach polar regions: do not trigger flipping
        } else {
            out.needsFlip = (pier != theoreticalPier);
        }

        // ETA: Strictly follow the continuous time-phase notation (positive if not passed, negative if passed) to avoid sudden jumps around the midpoint.
        out.etaMinutes = (-contHA) * 60.0;

        // Note: needsFlip has already been calculated above based on nearPole and the theoretical Pier
    }

    return out;
}


// void MainWindow::saveFitsAsJPG(QString filename)
// {
//     Logger::Log("Starting to save FITS as JPG...", LogLevel::INFO, DeviceType::GUIDER);
//     cv::Mat image;
//     cv::Mat image16;
//     cv::Mat SendImage;
//     Tools::readFits(filename.toLocal8Bit().constData(), image);
//     Logger::Log("FITS file read successfully.", LogLevel::INFO, DeviceType::GUIDER);

//     QList<FITSImage::Star> stars = Tools::FindStarsByStellarSolver(true, true);
//     Logger::Log("Star detection completed.", LogLevel::INFO, DeviceType::GUIDER);

//     if(stars.size() != 0){
//         FWHM = stars[0].HFR;
//         Logger::Log("FWHM calculated from detected stars.", LogLevel::INFO, DeviceType::GUIDER);
//     }
//     else {
//         FWHM = -1;
//         Logger::Log("No stars detected, FWHM set to -1.", LogLevel::WARNING, DeviceType::GUIDER);
//     }

//     if(image16.depth()==8) {
//         image.convertTo(image16, CV_16UC1, 256, 0); //x256  MSB alignment
//         Logger::Log("Image converted to 16-bit format with MSB alignment.", LogLevel::INFO, DeviceType::GUIDER);
//     }
//     else {
//         image.convertTo(image16, CV_16UC1, 1, 0);
//         Logger::Log("Image converted to 16-bit format.", LogLevel::INFO, DeviceType::GUIDER);
//     }

//     if(FWHM != -1){
//         // Draw detection results on the original image
//         cv::Point center(stars[0].x, stars[0].y);
//         cv::circle(image16, center, static_cast<int>(FWHM), cv::Scalar(0, 0, 255), 1); // Draw HFR circle
//         cv::circle(image16, center, 1, cv::Scalar(0, 255, 0), -1);                     // Draw center point
//         std::string hfrText = cv::format("%.2f", stars[0].HFR);
//         cv::putText(image16, hfrText, cv::Point(stars[0].x - FWHM, stars[0].y - FWHM - 5), cv::FONT_HERSHEY_SIMPLEX, 0.4, cv::Scalar(0, 0, 255), 1);
//         Logger::Log("Annotations for stars added to image.", LogLevel::INFO, DeviceType::GUIDER);
//     }

//     cv::Mat NewImage = image16;
//     FWHMCalOver = true;

//     cv::normalize(NewImage, SendImage, 0, 255, cv::NORM_MINMAX, CV_8U);    // Normalize new image
//     Logger::Log("Image normalized to 8-bit format.", LogLevel::INFO, DeviceType::GUIDER);

//     QString uniqueId = QUuid::createUuid().toString();
//     Logger::Log("Unique ID generated for new image.", LogLevel::INFO, DeviceType::GUIDER);

//     // List all files with the prefix 'CaptureImage'
//     QDir directory(QString::fromStdString(vueDirectoryPath));
//     QStringList filters;
//     filters << "CaptureImage*.jpg"; // Use a wildcard to filter jpg files with the prefix "CaptureImage"
//     QStringList fileList = directory.entryList(filters, QDir::Files);
//     Logger::Log("Existing image files listed for deletion.", LogLevel::INFO, DeviceType::GUIDER);

//     // Delete all matching files
//     for (const auto &file : fileList)
//     {
//         QString filePath = QString::fromStdString(vueDirectoryPath) + file;
//         QFile::remove(filePath);
//     }
//     Logger::Log("Old image files deleted.", LogLevel::INFO, DeviceType::GUIDER);

//     // Delete the previous image file
//     if (PriorROIImage != "NULL") {
//         QFile::remove(QString::fromStdString(PriorROIImage));
//         Logger::Log("Previous ROI image deleted.", LogLevel::INFO, DeviceType::GUIDER);
//     }

//     // Save the new image with a filename that has a unique ID
//     std::string fileName = "CaptureImage_" + uniqueId.toStdString() + ".jpg";
//     std::string filePath = vueDirectoryPath + fileName;
//     bool saved = cv::imwrite(filePath, SendImage);
//     Logger::Log("Attempt to save new image file.", LogLevel::INFO, DeviceType::GUIDER);

//     std::string Command = "sudo ln -sf " + filePath + " " + vueImagePath + fileName;
//     system(Command.c_str());
//     Logger::Log("Symbolic link created for new image file.", LogLevel::INFO, DeviceType::GUIDER);

//     PriorROIImage = vueImagePath + fileName;

//     if (saved)
//     {
//         emit wsThread->sendMessageToClient("SaveJpgSuccess:" + QString::fromStdString(fileName));

//         if(FWHM != -1){
//             dataPoints.append(QPointF(CurrentPosition, FWHM));

//             Logger::Log("dataPoints:" + std::to_string(CurrentPosition) + "," + std::to_string(FWHM), LogLevel::INFO, DeviceType::GUIDER);

//             float a, b, c;
//             Tools::fitQuadraticCurve(dataPoints, a, b, c);

//             if (dataPoints.size() >= 5) {
//                 QVector<QPointF> LineData;

//                 for (float x = CurrentPosition - 3000; x <= CurrentPosition + 3000; x += 10)
//                 {
//                     float y = a * x * x + b * x + c;
//                     LineData.append(QPointF(x, y));
//                 }

//                 // Calculate the x-coordinate where the derivative is zero
//                 float x_min = -b / (2 * a);
//                 minPoint_X = x_min;
//                 // Calculate the y-coordinate of the minimum point
//                 float y_min = a * x_min * x_min + b * x_min + c;

//                 QString dataString;
//                 for (const auto &point : LineData)
//                 {
//                     dataString += QString::number(point.x()) + "|" + QString::number(point.y()) + ":";
//                 }

//                 R2 = Tools::calculateRSquared(dataPoints, a, b, c);
//                 // qInfo() << "RSquared: " << R2;

//                 emit wsThread->sendMessageToClient("fitQuadraticCurve:" + dataString);
//                 emit wsThread->sendMessageToClient("fitQuadraticCurve_minPoint:" + QString::number(x_min) + ":" + QString::number(y_min));
//             }
//         }
//     }
//     else
//     {
//         Logger::Log("Failed to save image.", LogLevel::ERROR, DeviceType::GUIDER);
//     }
// }

int MainWindow::saveFitsAsPNG(QString fitsFileName, bool ProcessBin)
{
    if (false){
        fitsFileName = "/~/workspace/QUARCS/testimage1/1.fits";
    }
    Logger::Log("Starting to save FITS as PNG...", LogLevel::INFO, DeviceType::CAMERA);
    cv::Mat image;
    cv::Mat originalImage16;
    cv::Mat image16;
    Logger::Log("FITS file path: " + fitsFileName.toStdString(), LogLevel::INFO, DeviceType::CAMERA);
    int status = Tools::readFits(fitsFileName.toLocal8Bit().constData(), image);

    if (status != 0)
    {
        Logger::Log("Failed to read FITS file: " + fitsFileName.toStdString(), LogLevel::ERROR, DeviceType::CAMERA);
        return status;
    }
    if (image.type() == CV_8UC1 || image.type() == CV_8UC3 || image.type() == CV_16UC1)
    {
        originalImage16 = Tools::convert8UTo16U_BayerSafe(image, false);
        image.release();
    }
    else
    {
        Logger::Log("The current image data type is not supported for processing.", LogLevel::WARNING, DeviceType::CAMERA);
        return -1;
    }

    // Median blur
    Logger::Log("Starting median blur...", LogLevel::INFO, DeviceType::CAMERA);
    cv::medianBlur(originalImage16, originalImage16, 3);
    Logger::Log("Median blur applied successfully.", LogLevel::INFO, DeviceType::CAMERA);

    bool isColor = !(MainCameraCFA == "" || MainCameraCFA == "null");
    Logger::Log("Camera color mode: " + std::string(isColor ? "Color" : "Mono") + " CFA: " + MainCameraCFA.toStdString(), LogLevel::INFO, DeviceType::CAMERA);

    if (ProcessBin && glMainCameraBinning != 1)
    {
        // Use the PixelsDataSoftBin_Bayer function with the new Mat version
        if (MainCameraCFA == "RGGB" || MainCameraCFA == "RG")
        {
            image16 = Tools::PixelsDataSoftBin_Bayer(originalImage16, glMainCameraBinning, glMainCameraBinning, BAYER_RGGB);
        }
        else if (MainCameraCFA == "BGGR" || MainCameraCFA == "BG")
        {
            image16 = Tools::PixelsDataSoftBin_Bayer(originalImage16, glMainCameraBinning, glMainCameraBinning, BAYER_BGGR);
        }
        else if (MainCameraCFA == "GRBG" || MainCameraCFA == "GR")
        {
            image16 = Tools::PixelsDataSoftBin_Bayer(originalImage16, glMainCameraBinning, glMainCameraBinning, BAYER_GRBG);
        }
        else if (MainCameraCFA == "GBRG" || MainCameraCFA == "GB")
        {
            image16 = Tools::PixelsDataSoftBin_Bayer(originalImage16, glMainCameraBinning, glMainCameraBinning, BAYER_GBRG);
        }
        else
        {
            image16 = Tools::processMatWithBinAvg(originalImage16, glMainCameraBinning, glMainCameraBinning, isColor, true);
        }
    }
    else
    {
        image16 = originalImage16.clone();
    }
    originalImage16.release();

    Tools::SaveMatToFITS(image16);
    Logger::Log("Image saved as FITS.", LogLevel::INFO, DeviceType::CAMERA);

    int width = image16.cols;
    int height = image16.rows;
    Logger::Log("Image dimensions: " + std::to_string(width) + "x" + std::to_string(height), LogLevel::INFO, DeviceType::CAMERA);
    emit wsThread->sendMessageToClient("MainCameraSize:" + QString::number(width) + ":" + QString::number(height));
    if (ProcessBin)
    {
        emit wsThread->sendMessageToClient("MainCameraBinning:" + QString::number(glMainCameraBinning));
    }
    else
    {
        emit wsThread->sendMessageToClient("MainCameraBinning:" + QString::number(1));
    }

    std::vector<unsigned char> imageData; // uint16_t
    if (image16.type() == CV_16UC1)
    {
        imageData.assign(image16.data, image16.data + image16.total() * image16.channels() * 2);
    }
    else if (image16.type() == CV_8UC1)
    {
        imageData.assign(image16.data, image16.data + image16.total() * image16.channels());
    }
    else
    {
        Logger::Log("The current image data type is not supported for processing.", LogLevel::WARNING, DeviceType::CAMERA);
        return -1;
    }

    Logger::Log("Image data prepared for binary file.", LogLevel::INFO, DeviceType::CAMERA);

    QString uniqueId = QUuid::createUuid().toString();
    Logger::Log("Unique ID generated: " + uniqueId.toStdString(), LogLevel::INFO, DeviceType::CAMERA);

    QDir directory(QString::fromStdString(vueDirectoryPath));
    QStringList filters;
    filters << "CaptureImage*.bin";
    QStringList fileList = directory.entryList(filters, QDir::Files);
    Logger::Log("Existing binary files listed for deletion.", LogLevel::INFO, DeviceType::CAMERA);

    for (const auto &file : fileList)
    {
        QString filePath = QString::fromStdString(vueDirectoryPath) + file;
        QFile::remove(filePath);
    }
    Logger::Log("Old binary files deleted.", LogLevel::INFO, DeviceType::CAMERA);

    // Delete the previous image file
    if (PriorCaptureImage != "NULL")
    {
        QFile::remove(QString::fromStdString(PriorCaptureImage));
        Logger::Log("Previous capture image deleted.", LogLevel::INFO, DeviceType::CAMERA);
    }

    std::string fileName_ = "CaptureImage_" + uniqueId.toStdString() + ".bin";
    std::string filePath_ = vueDirectoryPath + fileName_;

    Logger::Log("Opening file for writing: " + filePath_, LogLevel::INFO, DeviceType::CAMERA);
    std::ofstream outFile(filePath_, std::ios::binary);
    if (!outFile)
    {
        Logger::Log("Failed to open file for writing.", LogLevel::ERROR, DeviceType::CAMERA);
        throw std::runtime_error("Failed to open file for writing.");
    }

    /Save the image size before writing to the file
    int showImageSizeX = width;
    int showImageSizeY = height;

    Logger::Log("Writing data to file...", LogLevel::INFO, DeviceType::CAMERA);
    outFile.write(reinterpret_cast<const char *>(imageData.data()), imageData.size());
    if (!outFile)
    {
        Logger::Log("Failed to write data to file.", LogLevel::ERROR, DeviceType::CAMERA);
        throw std::runtime_error("Failed to write data to file.");
    }

    outFile.close();
    if (!outFile)
    {
        Logger::Log("Failed to close the file properly.", LogLevel::ERROR, DeviceType::CAMERA);
        throw std::runtime_error("Failed to close the file properly.");
    }

    std::string Command = "sudo ln -sf " + filePath_ + " " + vueImagePath + fileName_;
    system(Command.c_str());
    Logger::Log("Symbolic link created for new image file.", LogLevel::INFO, DeviceType::CAMERA);

    PriorCaptureImage = vueImagePath + fileName_;
    emit wsThread->sendMessageToClient("SaveBinSuccess:" + QString::fromStdString(fileName_) + ":" + QString::number(showImageSizeX) + ":" + QString::number(showImageSizeY));
    isStagingImage = true;
    SavedImage = fileName_;
    Logger::Log("Binary image saved and client notified.", LogLevel::INFO, DeviceType::CAMERA);

    isSavePngSuccess = true;



    if (!fitsFileName.contains("ccd_simulator_original.fits"))
    {
        QString destinationPath = "/dev/shm/ccd_simulator_original.fits";
        QFile destinationFile(destinationPath);
        if (destinationFile.exists())
        {
            destinationFile.remove();
        }
        QFile::copy(fitsFileName, destinationPath);
    }
    if (isAutoFocus)
    {
        autoFocus->setCaptureComplete(fitsFileName);
    }



}
cv::Mat MainWindow::colorImage(cv::Mat img16)
{
    Logger::Log("Starting color image processing...", LogLevel::INFO, DeviceType::MAIN);
    // color camera, need to do debayer and color balance
    cv::Mat AWBImg16;
    cv::Mat AWBImg16color;
    cv::Mat AWBImg16mono;
    cv::Mat AWBImg8color;

    uint16_t B = 0;
    uint16_t W = 65535;

    AWBImg16.create(img16.rows, img16.cols, CV_16UC1);
    AWBImg16color.create(img16.rows, img16.cols, CV_16UC3);
    AWBImg16mono.create(img16.rows, img16.cols, CV_16UC1);
    AWBImg8color.create(img16.rows, img16.cols, CV_8UC3);

    Logger::Log("Matrices for image processing created.", LogLevel::INFO, DeviceType::MAIN);
    Tools::ImageSoftAWB(img16, AWBImg16, MainCameraCFA, ImageGainR, ImageGainB, 30); // image software Auto White Balance is done in RAW image.
    Logger::Log("Auto White Balance applied.", LogLevel::INFO, DeviceType::MAIN);
    cv::cvtColor(AWBImg16, AWBImg16color, cv::COLOR_BayerRG2BGR);
    Logger::Log("Image converted from Bayer to BGR.", LogLevel::INFO, DeviceType::MAIN);

    cv::cvtColor(AWBImg16color, AWBImg16mono, cv::COLOR_BGR2GRAY);
    Logger::Log("Image converted to grayscale.", LogLevel::INFO, DeviceType::MAIN);

    if (AutoStretch == true)
    {
        Tools::GetAutoStretch(AWBImg16mono, 0, B, W);
        Logger::Log("Auto stretch applied.", LogLevel::INFO, DeviceType::MAIN);
    }
    else
    {
        B = 0;
        W = 65535;
        Logger::Log("Auto stretch not applied, using default values.", LogLevel::INFO, DeviceType::MAIN);
    }
    Logger::Log("AutoStretch values: B=" + std::to_string(B) + ", W=" + std::to_string(W), LogLevel::INFO, DeviceType::MAIN);
    Tools::Bit16To8_Stretch(AWBImg16color, AWBImg8color, B, W);
    Logger::Log("Image stretched from 16-bit to 8-bit.", LogLevel::INFO, DeviceType::MAIN);

    AWBImg16.release();
    AWBImg16color.release();
    AWBImg16mono.release();
    AWBImg8color.release();
    Logger::Log("Temporary matrices released.", LogLevel::INFO, DeviceType::MAIN);

    return AWBImg16color;
}
void MainWindow::saveGuiderImageAsJPG(cv::Mat Image)
{
    Logger::Log("Starting to save guider image as JPG...", LogLevel::INFO, DeviceType::GUIDER);

    // Generate unique ID
    QString uniqueId = QUuid::createUuid().toString();
    Logger::Log("Generated unique ID for new guider image: " + uniqueId.toStdString(), LogLevel::INFO, DeviceType::GUIDER);

    // List all files with the prefix 'GuiderImage'
    QDir directory(QString::fromStdString(vueDirectoryPath));
    QStringList filters;
    filters << "GuiderImage*.jpg"; // Use a wildcard to filter jpg files with the prefix 'GuiderImage'
    QStringList fileList = directory.entryList(filters, QDir::Files);
    Logger::Log("Listed existing guider images for deletion.", LogLevel::INFO, DeviceType::GUIDER);

   // Delete all matching files
    for (const auto &file : fileList)
    {
        QString filePath = QString::fromStdString(vueDirectoryPath) + file;
        QFile::remove(filePath);
        Logger::Log("Deleted guider image file: " + filePath.toStdString(), LogLevel::INFO, DeviceType::GUIDER);
    }

   // Delete the previous image file
    if (PriorGuiderImage != "NULL")
    {
        QFile::remove(QString::fromStdString(PriorGuiderImage));
        Logger::Log("Deleted previous guider image file: " + PriorGuiderImage, LogLevel::INFO, DeviceType::GUIDER);
    }

   // Save the new image with a filename that has a unique ID
    std::string fileName = "GuiderImage_" + uniqueId.toStdString() + ".jpg";
    std::string filePath = vueDirectoryPath + fileName;
    bool saved = cv::imwrite(filePath, Image);
    Logger::Log("Attempted to save new guider image.", LogLevel::INFO, DeviceType::GUIDER);

    std::string Command = "sudo ln -sf " + filePath + " " + vueImagePath + fileName;
    system(Command.c_str());
    Logger::Log("Created symbolic link for new guider image.", LogLevel::INFO, DeviceType::GUIDER);

    PriorGuiderImage = vueImagePath + fileName;

    if (saved)
    {
        emit wsThread->sendMessageToClient(QString("GuideSize:%1:%2").arg(Image.cols).arg(Image.rows));
        emit wsThread->sendMessageToClient("SaveGuiderImageSuccess:" + QString::fromStdString(fileName));
        Logger::Log("Guider image saved successfully and client notified.", LogLevel::INFO, DeviceType::GUIDER);
    }
    else
    {
        Logger::Log("Failed to save guider image.", LogLevel::ERROR, DeviceType::GUIDER);
    }
}
void MainWindow::readDriversListFromFiles(const std::string &filename, DriversList &drivers_list_from,
                                          std::vector<DevGroup> &dev_groups_from, std::vector<Device> &devices_from)
{
    Logger::Log("Opening XML file: " + filename, LogLevel::INFO, DeviceType::GUIDER);
    QFile file(QString::fromStdString(filename));
    if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        Logger::Log("Failed to open file: " + filename, LogLevel::ERROR, DeviceType::GUIDER);
        return;
    }
    QXmlStreamReader xml(&file);
    while (!xml.atEnd() && !xml.hasError())
    {
        xml.readNext();
        if (xml.isStartElement() && xml.name() == "devGroup")
        {
            DevGroup dev_group;
            dev_group.group = xml.attributes().value("group").toString().toUtf8().constData();
            drivers_list_from.dev_groups.push_back(dev_group);
            Logger::Log("Added device group: " + dev_group.group.toStdString(), LogLevel::INFO, DeviceType::GUIDER);
        }
    }
    DIR *dir = opendir("/usr/share/indi");
    std::string DirPath = "/usr/share/indi/";
    std::string xmlpath;

    int index;

    DriversList drivers_list_get;
    std::vector<DevGroup> dev_groups_get;
    std::vector<Device> devices_get;

    DriversList drivers_list_xmls;
    DriversList drivers_list_xmls_null;
    std::vector<DevGroup> dev_groups_xmls;
    std::vector<Device> devices_xmls;

    std::vector<DevGroup> dev_groups;
    std::vector<Device> devices;

    if (dir == nullptr)
    {
        Logger::Log("Unable to find INDI drivers directory at /usr/share/indi", LogLevel::ERROR, DeviceType::GUIDER);
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != nullptr)
    {
        if (strcmp(entry->d_name + strlen(entry->d_name) - 4, ".xml") == 0)
        {
            if (strcmp(entry->d_name + strlen(entry->d_name) - 6, "sk.xml") == 0)
            {
                continue; // Skip sky charts
            }
            else
            {
                xmlpath = DirPath + entry->d_name;
                QFile file(QString::fromStdString(xmlpath));
                if (!file.open(QIODevice::ReadOnly | QIODevice::Text))
                {
                    Logger::Log("Failed to open file: " + xmlpath, LogLevel::ERROR, DeviceType::GUIDER);
                }

                QXmlStreamReader xml(&file);

                while (!xml.atEnd() && !xml.hasError())
                {
                    xml.readNext();
                    if (xml.isStartElement() && xml.name() == "devGroup")
                    {
                        DevGroup dev_group;
                        dev_group.group = xml.attributes().value("group").toString().toUtf8().constData();
                        dev_groups.push_back(dev_group);
                        while (!(xml.isEndElement() && xml.name() == "devGroup"))
                        {
                            xml.readNext();
                            if (xml.isStartElement() && xml.name() == "device")
                            {
                                Device device;
                                device.label = xml.attributes().value("label").toString().toStdString();

                                device.manufacturer = xml.attributes().value("manufacturer").toString().toStdString();
                                devices.push_back(device);
                                while (!(xml.isEndElement() && xml.name() == "device"))
                                {
                                    xml.readNext();
                                    if (xml.isStartElement() && xml.name() == "driver")
                                    {
                                        device.driver_name = xml.readElementText().toStdString();
                                    }
                                    else if (xml.isStartElement() && xml.name() == "version")
                                    {
                                        device.version = xml.readElementText().toStdString();
                                    }
                                }
                                dev_group.devices.push_back(device);
                            }
                        }
                        drivers_list_xmls.dev_groups.push_back(dev_group);
                    }
                }
            }
        }
        for (int i = 0; i < drivers_list_xmls.dev_groups.size(); i++)
        {
            for (int j = 0; j < drivers_list_from.dev_groups.size(); j++)
            {
                if (drivers_list_xmls.dev_groups[i].group == drivers_list_from.dev_groups[j].group)
                {
                    for (int k = 0; k < drivers_list_xmls.dev_groups[i].devices.size(); k++)
                    {
                        Device dev;
                        dev.driver_name = drivers_list_xmls.dev_groups[i].devices[k].driver_name;
                        dev.label = drivers_list_xmls.dev_groups[i].devices[k].label;
                        dev.version = drivers_list_xmls.dev_groups[i].devices[k].version;
                        drivers_list_from.dev_groups[j].devices.push_back(dev);
                    }
                }
            }
        }
        drivers_list_xmls = drivers_list_xmls_null;
    }
    closedir(dir);
    Logger::Log("Completed reading and processing INDI driver files.", LogLevel::INFO, DeviceType::GUIDER);
}

//"Telescopes"|"Focusers"|"CCDs"|"Spectrographs"|"Filter Wheels"|"Auxiliary"|"Domes"|"Weather"|"Agent"
void MainWindow::printDevGroups2(const DriversList drivers_list, int ListNum, QString group)
{
    Logger::Log("Printing device groups for group: " + group.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("=============================== Print DevGroups ===============================", LogLevel::INFO, DeviceType::MAIN);
    for (int i = 0; i < drivers_list.dev_groups.size(); i++)
    {
        if (drivers_list.dev_groups[i].group == group)
        {
            Logger::Log("Processing device group: " + drivers_list.dev_groups[i].group.toStdString(), LogLevel::INFO, DeviceType::MAIN);
            // Uncomment and modify the following lines if you want to log device details and send messages
             for (int j = 0; j < drivers_list.dev_groups[i].devices.size(); j++)
             {
                 qDebug() << QString::fromStdString(drivers_list.dev_groups[i].devices[j].driver_name) << QString::fromStdString(drivers_list.dev_groups[i].devices[j].version) << QString::fromStdString(drivers_list.dev_groups[i].devices[j].label);
                 Logger::Log("Device details: " + drivers_list.dev_groups[i].devices[j].label + ", " + drivers_list.dev_groups[i].devices[j].driver_name + ", " + drivers_list.dev_groups[i].devices[j].version, LogLevel::INFO, DeviceType::MAIN);
                 emit wsThread->sendMessageToClient("AddDriver:" + QString::fromStdString(drivers_list.dev_groups[i].devices[j].label) + ":" + QString::fromStdString(drivers_list.dev_groups[i].devices[j].driver_name));
                 //wsThread->sendMessageToClient("AddDriver:"+QString::fromStdString(drivers_list.dev_groups[i].devices[j].label)+":"+QString::fromStdString(drivers_list.dev_groups[i].devices[j].driver_name));
             }
            DeviceSelect(ListNum, i);
        }
    }
    Logger::Log("Completed printing device groups.", LogLevel::INFO, DeviceType::MAIN);
}

void MainWindow::DeviceSelect(int systemNumber, int grounpNumber)
{
    // Tools::clearSystemDeviceListItem(systemdevicelist, systemNumber);
    SelectIndiDevice(systemNumber, grounpNumber);
}

void MainWindow::SelectIndiDevice(int systemNumber, int grounpNumber)
{
    systemdevicelist.currentDeviceCode = systemNumber;
    drivers_list.selectedGrounp = grounpNumber;

    // switch (systemNumber)
    // {
    // case 0:
    //     systemdevicelist.system_devices[systemNumber].Description = "Mount";
    //     break;
    // case 1:
    //     systemdevicelist.system_devices[systemNumber].Description = "Guider";
    //     break;
    // case 2:
    //     systemdevicelist.system_devices[systemNumber].Description = "PoleCamera";
    //     break;
    // case 20:
    //     systemdevicelist.system_devices[systemNumber].Description = "Main Camera #1";
    //     break;
    // case 21:
    //     systemdevicelist.system_devices[systemNumber].Description = "CFW #1";
    //     break;
    // case 22:
    //     systemdevicelist.system_devices[systemNumber].Description = "Focuser #1";
    //     break;

    // default:
    //     break;
    // }

    // qDebug() << "SelectIndiDevice:" << systemdevicelist.currentDeviceCode << "," << drivers_list.selectedGrounp;

    for (int i = 0; i < drivers_list.dev_groups[grounpNumber].devices.size(); i++)
    {
        if (grounpNumber == 1 && (QString::fromStdString(drivers_list.dev_groups[grounpNumber].devices[i].label) == "QHY CCD" || QString::fromStdString(drivers_list.dev_groups[grounpNumber].devices[i].driver_name) == "indi_qhy_ccd"))
        {
            continue;
        }
        if (grounpNumber == 20 && (QString::fromStdString(drivers_list.dev_groups[grounpNumber].devices[i].label) == "QHY CCD2" || QString::fromStdString(drivers_list.dev_groups[grounpNumber].devices[i].driver_name) == "indi_qhy_ccd2"))
        {
            continue;
        }
        // qDebug() << QString::fromStdString(drivers_list.dev_groups[grounpNumber].devices[i].driver_name) << QString::fromStdString(drivers_list.dev_groups[grounpNumber].devices[i].version) << QString::fromStdString(drivers_list.dev_groups[grounpNumber].devices[i].label) << QString::fromStdString(drivers_list.dev_groups[grounpNumber].devices[i].manufacturer);
        emit wsThread->sendMessageToClient("AddDriver:" + QString::fromStdString(drivers_list.dev_groups[grounpNumber].devices[i].label) + ":" + QString::fromStdString(drivers_list.dev_groups[grounpNumber].devices[i].driver_name));
        // qDebug() << "AddDriver:" + QString::fromStdString(drivers_list.dev_groups[grounpNumber].devices[i].label) + ":" + QString::fromStdString(drivers_list.dev_groups[grounpNumber].devices[i].driver_name);
    }
}

bool MainWindow::indi_Driver_Confirm(QString DriverName, QString BaudRate)
{
    switch (systemdevicelist.currentDeviceCode)
    {
    case 0:
        // Fix: Check if the currentDeviceCode index is valid
        if (systemdevicelist.system_devices.size() > systemdevicelist.currentDeviceCode) {
            systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].Description = "Mount";
            systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].BaudRate = BaudRate.toInt();
            Logger::Log("indi_Driver_Confirm | Mount | DriverName: " + DriverName.toStdString() + " BaudRate: " + std::to_string(BaudRate.toInt()), LogLevel::INFO, DeviceType::MAIN);
        }
        
        break;
    case 1:
        if (systemdevicelist.system_devices.size() > systemdevicelist.currentDeviceCode) {
            systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].Description = "Guider";
            systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].BaudRate = BaudRate.toInt();
            Logger::Log("indi_Driver_Confirm | Guider | DriverName: " + DriverName.toStdString() + " BaudRate: " + std::to_string(BaudRate.toInt()), LogLevel::INFO, DeviceType::MAIN);
        }
        break;
    case 2:
        if (systemdevicelist.system_devices.size() > systemdevicelist.currentDeviceCode) {
            systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].Description = "PoleCamera";
            systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].BaudRate = BaudRate.toInt();
            Logger::Log("indi_Driver_Confirm | PoleCamera | DriverName: " + DriverName.toStdString() + " BaudRate: " + std::to_string(BaudRate.toInt()), LogLevel::INFO, DeviceType::MAIN);
        }
        break;
    case 20:
        if (systemdevicelist.system_devices.size() > systemdevicelist.currentDeviceCode) {
            systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].Description = "MainCamera";
            systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].BaudRate = BaudRate.toInt();
            Logger::Log("indi_Driver_Confirm | MainCamera | DriverName: " + DriverName.toStdString() + " BaudRate: " + std::to_string(BaudRate.toInt()), LogLevel::INFO, DeviceType::MAIN);
        }   
        break;
    case 21:
        if (systemdevicelist.system_devices.size() > systemdevicelist.currentDeviceCode) {
            systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].Description = "CFW";
            systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].BaudRate = BaudRate.toInt();
            Logger::Log("indi_Driver_Confirm | CFW | DriverName: " + DriverName.toStdString() + " BaudRate: " + std::to_string(BaudRate.toInt()), LogLevel::INFO, DeviceType::MAIN);
        }
        break;
    case 22:
        if (systemdevicelist.system_devices.size() > systemdevicelist.currentDeviceCode) {
            systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].Description = "Focuser";
            systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].BaudRate = BaudRate.toInt();
            Logger::Log("indi_Driver_Confirm | Focuser | DriverName: " + DriverName.toStdString() + " BaudRate: " + std::to_string(BaudRate.toInt()), LogLevel::INFO, DeviceType::MAIN);
        }
        break;

    default:
        Logger::Log("indi_Driver_Confirm | Invalid currentDeviceCode: " + std::to_string(systemdevicelist.currentDeviceCode), LogLevel::ERROR, DeviceType::MAIN);
        break;
    }

    // Fix: check index validity before accessing
    if (systemdevicelist.system_devices.size() > systemdevicelist.currentDeviceCode) {
        systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].DriverIndiName = DriverName;
    } else {
        Logger::Log("indi_Driver_Confirm | currentDeviceCode out of bounds: " + std::to_string(systemdevicelist.currentDeviceCode), LogLevel::ERROR, DeviceType::MAIN);
    }
    return false;
}

bool MainWindow::indi_Driver_Clear()
{
    // Fix: check index validity
    if (systemdevicelist.system_devices.size() > systemdevicelist.currentDeviceCode) {
        systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].Description = "";
        systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].DriverIndiName = "";
        systemdevicelist.system_devices[systemdevicelist.currentDeviceCode].BaudRate = 9600;
    } else {
        Logger::Log("indi_Driver_Clear | currentDeviceCode out of bounds: " + std::to_string(systemdevicelist.currentDeviceCode), LogLevel::ERROR, DeviceType::MAIN);
        return false;
    }
    return true;
}

void MainWindow::indi_Device_Confirm(QString DeviceName, QString DriverName)
{
    //   qApp->processEvents();

    int deviceCode;
    deviceCode = systemdevicelist.currentDeviceCode;

    systemdevicelist.system_devices[deviceCode].DriverIndiName = DriverName;
    systemdevicelist.system_devices[deviceCode].DeviceIndiGroup = drivers_list.selectedGrounp;
    systemdevicelist.system_devices[deviceCode].DeviceIndiName = DeviceName;

    Logger::Log("system device(" + DeviceName.toStdString() + ") successfully selected", LogLevel::INFO, DeviceType::MAIN);

    Tools::printSystemDeviceList(systemdevicelist);

    Tools::saveSystemDeviceList(systemdevicelist);
}

uint32_t MainWindow::clearCheckDeviceExist(QString drivername, bool &isExist)
{
    Logger::Log("Stopping all INDI drivers.", LogLevel::INFO, DeviceType::MAIN);
    Tools::stopIndiDriverAll(drivers_list);
    Logger::Log("Starting INDI driver: " + drivername.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    Tools::startIndiDriver(drivername);

    sleep(1); // must wait some time here

    MyClient *searchClient;
    searchClient = new MyClient();
    Logger::Log("Initialized new MyClient for device search.", LogLevel::INFO, DeviceType::MAIN);
    searchClient->PrintDevices();

    searchClient->setServer("localhost", 7624);
    searchClient->setConnectionTimeout(3, 0);
    searchClient->ClearDevices(); // clear device list

    Logger::Log("Attempting to connect to INDI server at localhost:7624", LogLevel::INFO, DeviceType::MAIN);
    bool connected = searchClient->connectServer();

    if (connected == false)
    {
        Logger::Log("Failed to connect to INDI server, can not find server", LogLevel::ERROR, DeviceType::MAIN);
        return QHYCCD_ERROR;
    }

    sleep(1); // connect server will generate the callback of newDevice and then put the device into list. this need take some time and it is non-block
    searchClient->PrintDevices();

    if (searchClient->GetDeviceCount() == 0)
    {
        Logger::Log("No devices found on INDI server.", LogLevel::INFO, DeviceType::MAIN);
        searchClient->disconnectServer();
        isExist = false;
        emit wsThread->sendMessageToClient("ScanFailed:No device found.");
        return QHYCCD_SUCCESS;
    }

    Logger::Log("Devices found: " + std::to_string(searchClient->GetDeviceCount()), LogLevel::INFO, DeviceType::MAIN);
    for (int i = 0; i < searchClient->GetDeviceCount(); i++)
    {
        emit wsThread->sendMessageToClient("AddDevice:" + QString::fromStdString(searchClient->GetDeviceNameFromList(i)));
    }

    searchClient->disconnectServer();
    searchClient->ClearDevices();

    Tools::stopIndiDriver(drivername);
    Logger::Log("INDI driver stopped: " + drivername.toStdString(), LogLevel::INFO, DeviceType::MAIN);

    return QHYCCD_SUCCESS;
}

void MainWindow::ConnectAllDeviceOnce()
{
    Logger::Log("Connecting all devices once.", LogLevel::INFO, DeviceType::MAIN);
    
    // Defensive check: Ensure indi_Client has been initialized
    if (indi_Client == nullptr)
    {
        Logger::Log("ConnectAllDeviceOnce | indi_Client is nullptr", LogLevel::ERROR, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConnectFailed:ClientNotInitialized");
        return;
    }

    dpMount = nullptr;
    dpGuider = nullptr;
    dpPoleScope = nullptr;
    dpMainCamera = nullptr;
    dpFocuser = nullptr;
    dpCFW = nullptr;

    int SelectedDriverNum = Tools::getDriverNumFromSystemDeviceList(systemdevicelist);
    if (SelectedDriverNum == 0)
    {
        Logger::Log("No driver in system device list.", LogLevel::ERROR, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConnectFailed:No driver in system device list.");
        return;
    }

    // NumberOfTimesConnectDevice = 0;

    Tools::cleanSystemDeviceListConnect(systemdevicelist);
    disconnectIndiServer(indi_Client);
    Tools::stopIndiDriverAll(drivers_list);

    QString driverName;
    QString deviceType;
    QVector<QString> nameCheck;

    for (int i = 0; i < systemdevicelist.system_devices.size(); i++)
    {
        driverName = systemdevicelist.system_devices[i].DriverIndiName;
        deviceType = systemdevicelist.system_devices[i].Description;

        if (driverName != "")
        {
            bool isFound = false;
            for (auto item : nameCheck)
            {
                if ((item == driverName) || (item == "indi_qhy_ccd" && driverName == "indi_qhy_ccd2") || (item == "indi_qhy_ccd2" && driverName == "indi_qhy_ccd"))
                {

                    isFound = true;
                    Logger::Log("Found one duplite driver,do not start it again: " + driverName.toStdString(), LogLevel::INFO, DeviceType::MAIN);
                    break;
                }
            }

            if (isFound == false)
            {
                Logger::Log("Start INDI Driver:" + driverName.toStdString(), LogLevel::INFO, DeviceType::MAIN);
                Tools::startIndiDriver(driverName);
                nameCheck.push_back(driverName);
                ConnectDriverList.push_back(driverName);
            }
        }
    }

    sleep(1);

    // Defensive check again to avoid null pointer dereference
    if (indi_Client == nullptr)
    {
        Logger::Log("ConnectAllDeviceOnce | indi_Client became nullptr before server check", LogLevel::ERROR, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConnectFailed:ClientDisconnected");
        return;
    }

    if (indi_Client->isServerConnected() == false)
    {
        Logger::Log("Can not find server.", LogLevel::ERROR, DeviceType::MAIN);
        connectIndiServer(indi_Client);
        sleep(1);
    }

    QTimer *timer = new QTimer(this);
    timer->setInterval(1000); // Set the timer interval to 1000 milliseconds
    int time = 0;
    connect(timer, &QTimer::timeout, this, [this, timer, &time]()
            {
        // Defensive check: prevent segmentation fault caused by indi_Client being null
        if (indi_Client == nullptr) {
            Logger::Log("ConnectAllDeviceOnce | indi_Client is nullptr in timer callback", LogLevel::ERROR, DeviceType::MAIN);
            timer->stop();
            timer->deleteLater();
            emit wsThread->sendMessageToClient("ConnectFailed:ClientNotInitialized");
            return;
        }

        if (indi_Client->GetDeviceCount() > 0 || time >= 10) {
            timer->stop();
            timer->deleteLater();
            sleep(2);
            continueConnectAllDeviceOnce(); // Continue executing the remaining part of the device connection
        } else {
            Logger::Log("Waiting for devices...", LogLevel::INFO, DeviceType::MAIN);
            time++;
        } });
    timer->start();
}
void MainWindow::continueConnectAllDeviceOnce()
{
    // Defensive check: Ensure indi_Client is valid
    if (indi_Client == nullptr)
    {
        Logger::Log("continueConnectAllDeviceOnce | indi_Client is nullptr", LogLevel::ERROR, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConnectFailed:ClientNotInitialized");
        Tools::stopIndiDriverAll(drivers_list);
        ConnectDriverList.clear();
        return;
    }

    if (indi_Client->GetDeviceCount() == 0)
    {
        Logger::Log("Driver start success but no device found", LogLevel::ERROR, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConnectFailed:No device found.");
        Tools::stopIndiDriverAll(drivers_list);
        ConnectDriverList.clear();
        return;
    }

    // Helper function: Find the corresponding SystemDevice slot based on the INDI device and return the baud rate to use
    auto getBaudRateForDeviceIndex = [this](INDI::BaseDevice *device, int deviceIndex) -> int
    {
        int defaultBaud = 9600;

        // First, use the original index-based logic (to maintain compatibility)
        if (deviceIndex >= 0 && deviceIndex < systemdevicelist.system_devices.size())
        {
            defaultBaud = systemdevicelist.system_devices[deviceIndex].BaudRate;
        }

        if (device == nullptr)
            return defaultBaud;

        // Try again to match exactly in systemdevicelist based on the driver name
        QString driverExec = QString::fromUtf8(device->getDriverExec());
        for (int idx = 0; idx < systemdevicelist.system_devices.size(); idx++)
        {
            if (systemdevicelist.system_devices[idx].DriverIndiName == driverExec)
            {
                return systemdevicelist.system_devices[idx].BaudRate;
            }
        }

        return defaultBaud;
    };

    for (int i = 0; i < indi_Client->GetDeviceCount(); i++)
    {
        // Fix: Check if the system device list index is valid
        if (i >= systemdevicelist.system_devices.size()) {
            Logger::Log("ConnectAllDeviceOnce | Index " + std::to_string(i) + " out of bounds for systemdevicelist (size: " + std::to_string(systemdevicelist.system_devices.size()) + ")", LogLevel::ERROR, DeviceType::MAIN);
            break; // Stop the loop to prevent out-of-bounds access
        }
        
        // Fix: Check if the device pointer is valid
        INDI::BaseDevice *device = indi_Client->GetDeviceFromList(i);
        if (device == nullptr) {
            Logger::Log("ConnectAllDeviceOnce | Device at index " + std::to_string(i) + " is nullptr", LogLevel::WARNING, DeviceType::MAIN);
            continue; // Skip this device
        }
        
        std::string deviceName = indi_Client->GetDeviceNameFromList(i);
        if (deviceName.empty()) {
            Logger::Log("ConnectAllDeviceOnce | Device name at index " + std::to_string(i) + " is empty", LogLevel::WARNING, DeviceType::MAIN);
            continue;
        }
        
        Logger::Log("Start connecting devices:" + deviceName, LogLevel::INFO, DeviceType::MAIN);

        // Before establishing a formal connection, apply override settings only when the user manually selects the serial port; do not change the port in default mode
        // Find the corresponding DriverType (Mount / Focuser) in systemdevicelist according to driverExec）
        QString driverExec = QString::fromUtf8(device->getDriverExec());
        QString driverType;
        for (int idx = 0; idx < systemdevicelist.system_devices.size(); idx++)
        {
            if (systemdevicelist.system_devices[idx].DriverIndiName == driverExec)
            {
                driverType = systemdevicelist.system_devices[idx].Description;
                break;
            }
        }
        if (driverType == "Focuser" && !focuserSerialPortOverride.isEmpty())
        {
            indi_Client->setDevicePort(device, focuserSerialPortOverride);
            Logger::Log("ConnectAllDeviceOnce | Focuser initial Port set to: " + focuserSerialPortOverride.toStdString(),
                        LogLevel::INFO, DeviceType::MAIN);

        // After setting the serial port based on the override value, sync the current serial port with the candidate list to the frontend
        // This way, the serial port dropdown on the frontend will immediately show the actual used/overridden port
            sendSerialPortOptions(driverType);
        }
        else if (driverType == "Mount" && !mountSerialPortOverride.isEmpty())
        {
            indi_Client->setDevicePort(device, mountSerialPortOverride);
            Logger::Log("ConnectAllDeviceOnce | Mount initial Port set to: " + mountSerialPortOverride.toStdString(),
                        LogLevel::INFO, DeviceType::MAIN);

        // After setting the serial port based on the override value, synchronize the current serial port and the candidate list to the frontend
            sendSerialPortOptions(driverType);
        }

        int baudRateToUse = getBaudRateForDeviceIndex(device, i);
        Logger::Log("ConnectAllDeviceOnce | setBaudRate for device " + deviceName + " -> " + std::to_string(baudRateToUse),
                    LogLevel::INFO, DeviceType::MAIN);
        indi_Client->setBaudRate(device, baudRateToUse);
        indi_Client->connectDevice(deviceName.c_str());

        int waitTime = 0;
        while (device != nullptr && !device->isConnected() && waitTime < 5)
        {
            Logger::Log("Wait for Connect" + deviceName, LogLevel::INFO, DeviceType::MAIN);
            QThread::msleep(1000); // wait 1 second
            waitTime++;
        }

        if (device == nullptr || !device->isConnected())
        {
            Logger::Log("ConnectDriver | Device (" + deviceName + ") is not connected,try to update port", LogLevel::WARNING, DeviceType::MAIN);
            // Special handling (focuser and mount)
            // Fix: use the verified device pointer to avoid repeated GetDeviceFromList calls
            if (device != nullptr && (device->getDriverInterface() & INDI::BaseDevice::FOCUSER_INTERFACE || device->getDriverInterface() & INDI::BaseDevice::TELESCOPE_INTERFACE)){
                QString DevicePort;
                indi_Client->getDevicePort(device, DevicePort);
                QString DeviceType = detector.detectDeviceTypeForPort(DevicePort);

                // Get device type
                QString DriverType = "";
                // Fix: use the verified device pointer
                if (device != nullptr) {
                    for(int j = 0; j < systemdevicelist.system_devices.size(); j++)
                    {
                        if (systemdevicelist.system_devices[j].DriverIndiName == device->getDriverExec())
                        {
                            DriverType = systemdevicelist.system_devices[j].Description;
                        }
                    }
                    // Handle connections for focuser and mount
                    if (DeviceType != "Focuser" && DriverType == "Focuser")
                    {
                        // Detected device is a focuser but its serial port is not the focuser port; needs updating
                        // The correct port is detector.getFocuserPort()
                        QString realFocuserPort = detector.getFocuserPort();
                        if (!realFocuserPort.isEmpty())
                        {
                            indi_Client->setDevicePort(device, realFocuserPort);
                            // Update override value synchronously to keep subsequent connections and frontend display consistent
                            focuserSerialPortOverride = realFocuserPort;
                            Logger::Log("ConnectDriver | Focuser Device (" + std::string(device->getDeviceName()) + ") Port is updated to: " + realFocuserPort.toStdString(), LogLevel::INFO, DeviceType::MAIN);

                            // After auto-correcting the port, sync the current port with the candidate list to the frontend
                            sendSerialPortOptions(DriverType);
                        }
                        else
                        {
                            Logger::Log("No matched Focuser port found by detector.", LogLevel::WARNING, DeviceType::MAIN);
                            continue;
                        }
                    }else if (DeviceType != "Mount" && DriverType == "Mount")
                    {
                        // Detected device is a mount but its serial port is not the mount port; needs updating
                        // The correct port is detector.getMountPort()
                        QString realMountPort = detector.getMountPort();
                        if (!realMountPort.isEmpty())
                        {
                            indi_Client->setDevicePort(device, realMountPort);
                            // Update override value synchronously to keep subsequent connections and frontend display consistent
                            mountSerialPortOverride = realMountPort;
                            Logger::Log("ConnectDriver | Mount Device (" + std::string(device->getDeviceName()) + ") Port is updated to: " + realMountPort.toStdString(), LogLevel::INFO, DeviceType::MAIN);

                            // After auto-correcting the port, sync the current port with the candidate list to the frontend
                            sendSerialPortOptions(DriverType);
                        }
                        else
                        {
                            Logger::Log("No matched Mount port found by detector.", LogLevel::WARNING, DeviceType::MAIN);
                            continue;
                        }
                    }else{
                        Logger::Log("ConnectDriver | Device (" + std::string(device->getDeviceName()) + ") Port is not updated.", LogLevel::WARNING, DeviceType::MAIN);
                    }
                }
            }
            // Fix: use the verified device pointer and deviceName
            if (device != nullptr && !deviceName.empty()) {
                int retryBaudRate = getBaudRateForDeviceIndex(device, i);
                Logger::Log("ConnectAllDeviceOnce | retry setBaudRate for device " + deviceName + " -> " + std::to_string(retryBaudRate),
                            LogLevel::INFO, DeviceType::MAIN);
                indi_Client->setBaudRate(device, retryBaudRate);
                indi_Client->connectDevice(deviceName.c_str());
        
                int waitTime = 0;
                while (device != nullptr && !device->isConnected() && waitTime < 5)
                {
                    // Fix: use the verified deviceName variable
                    Logger::Log("Wait for Connect" + deviceName, LogLevel::INFO, DeviceType::MAIN);
                    QThread::msleep(1000); // wait 1 second
                    waitTime++;
                }
            }
        }
    }


    ConnectedCCDList.clear();
    ConnectedTELESCOPEList.clear();
    ConnectedFOCUSERList.clear();
    ConnectedFILTERList.clear();
    for (int i = 0; i < indi_Client->GetDeviceCount(); i++) //  indi_Client->GetDeviceFromList(i)
    {
        // 修复：检查设备指针是否有效
        INDI::BaseDevice *device = indi_Client->GetDeviceFromList(i);
        if (device == nullptr) {
            Logger::Log("AfterDeviceConnect | Device at index " + std::to_string(i) + " is nullptr", LogLevel::WARNING, DeviceType::MAIN);
            continue;
        }
        
        if (device->isConnected())
        {
            if (device->getDriverInterface() & INDI::BaseDevice::CCD_INTERFACE)
            {
                Logger::Log("We received a CCD!", LogLevel::INFO, DeviceType::MAIN);
                ConnectedCCDList.push_back(i);
            }
            else if (device->getDriverInterface() & INDI::BaseDevice::FILTER_INTERFACE)
            {
                Logger::Log("We received a FILTER!", LogLevel::INFO, DeviceType::MAIN);
                ConnectedFILTERList.push_back(i);
            }
            else if (device->getDriverInterface() & INDI::BaseDevice::TELESCOPE_INTERFACE)
            {
                Logger::Log("We received a TELESCOPE!", LogLevel::INFO, DeviceType::MAIN);
                ConnectedTELESCOPEList.push_back(i);
            }
            else if (device->getDriverInterface() & INDI::BaseDevice::FOCUSER_INTERFACE)
            {
                Logger::Log("We received a FOCUSER!", LogLevel::INFO, DeviceType::MAIN);
                ConnectedFOCUSERList.push_back(i);
            }
            Logger::Log("Driver:" + std::string(device->getDriverExec()) + " Device:" + std::string(device->getDeviceName()), LogLevel::INFO, DeviceType::MAIN);
        }
        else
        {
            QString DeviceName = QString::fromStdString(device->getDeviceName());
            Logger::Log("Connect failed device:" + DeviceName.toStdString(), LogLevel::ERROR, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("ConnectFailed:Connect device failed:" + DeviceName);
        }
    }

    Tools::printSystemDeviceList(systemdevicelist);

    QStringList SelectedCameras = Tools::getCameraNumFromSystemDeviceList(systemdevicelist);
    Logger::Log("Number of Selected cameras:" + std::to_string(SelectedCameras.size()), LogLevel::INFO, DeviceType::MAIN);
    for (auto Camera : SelectedCameras)
    {
        Logger::Log("Selected Cameras:" + Camera.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    }

    Logger::Log("Number of Connected CCD:" + std::to_string(ConnectedCCDList.size()), LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("Number of Connected TELESCOPE:" + std::to_string(ConnectedTELESCOPEList.size()), LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("Number of Connected FOCUSER:" + std::to_string(ConnectedFOCUSERList.size()), LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("Number of Connected FILTER:" + std::to_string(ConnectedFILTERList.size()), LogLevel::INFO, DeviceType::MAIN);

    for (int i = 0; i < indi_Client->GetDeviceCount(); i++)
    {
        // 修复：检查设备指针是否有效
        INDI::BaseDevice *device = indi_Client->GetDeviceFromList(i);
        if (device == nullptr) {
            continue;
        }
        
        if (device->isConnected())
        {
            std::string driverExec = device->getDriverExec();
            QString driverExecQString = QString::fromStdString(driverExec);
            for (int j = 0; j < systemdevicelist.system_devices.size(); j++)
            {
                if (systemdevicelist.system_devices[j].DriverIndiName == driverExecQString || 
                    (systemdevicelist.system_devices[j].DriverIndiName == "indi_qhy_ccd" && driverExec == "indi_qhy_ccd2") || 
                    (systemdevicelist.system_devices[j].DriverIndiName == "indi_qhy_ccd2" && driverExec == "indi_qhy_ccd"))
                {
                    emit wsThread->sendMessageToClient("AddDeviceType:" + systemdevicelist.system_devices[j].Description);
                }
            }
        }
    }

    if (ConnectedCCDList.size() <= 0 && ConnectedTELESCOPEList.size() <= 0 && ConnectedFOCUSERList.size() <= 0 && ConnectedFILTERList.size() <= 0)
    {
        Logger::Log("No Device Connected", LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConnectFailed:No Device Connected");
        Tools::stopIndiDriverAll(drivers_list);
        ConnectDriverList.clear();
        return;
    }

    bool EachDeviceOne = true;

    if (SelectedCameras.size() == 1 && ConnectedCCDList.size() == 1)
    {
        Logger::Log("The Camera Selected and Connected are Both 1", LogLevel::INFO, DeviceType::MAIN);
        // 修复：检查向量是否为空以及索引是否有效
        if (SelectedCameras.empty() || ConnectedCCDList.empty()) {
            Logger::Log("SelectedCameras or ConnectedCCDList is empty, cannot assign camera", LogLevel::ERROR, DeviceType::MAIN);
        } else if (SelectedCameras[0] == "Guider")
        {
            INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedCCDList[0]);
            if (device == nullptr) {
                Logger::Log("GetDeviceFromList returned nullptr for ConnectedCCDList[0]", LogLevel::ERROR, DeviceType::MAIN);
            } else {
                dpGuider = device;
                // 修复：检查system_devices索引是否有效
                if (systemdevicelist.system_devices.size() > 1) {
                    systemdevicelist.system_devices[1].isConnect = true;
                }
                indi_Client->disconnectDevice(device->getDeviceName());
                sleep(1);
                call_phd_whichCamera(device->getDeviceName());
                // PHD2 connect status
                AfterDeviceConnect(dpGuider);
            }
        }
        else if (SelectedCameras[0] == "PoleCamera")
        {
            INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedCCDList[0]);
            if (device == nullptr) {
                Logger::Log("GetDeviceFromList returned nullptr for ConnectedCCDList[0]", LogLevel::ERROR, DeviceType::MAIN);
            } else {
                dpPoleScope = device;
                if (systemdevicelist.system_devices.size() > 2) {
                    systemdevicelist.system_devices[2].isConnect = true;
                }
                AfterDeviceConnect(dpPoleScope);
            }
        }
        else if (SelectedCameras[0] == "MainCamera")
        {
            INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedCCDList[0]);
            if (device == nullptr) {
                Logger::Log("GetDeviceFromList returned nullptr for ConnectedCCDList[0]", LogLevel::ERROR, DeviceType::MAIN);
            } else {
                dpMainCamera = device;
                if (systemdevicelist.system_devices.size() > 20) {
                    systemdevicelist.system_devices[20].isConnect = true;
                }
                AfterDeviceConnect(dpMainCamera);
            }
        }
    }
    else if (SelectedCameras.size() > 1 || ConnectedCCDList.size() > 1)
    {
        EachDeviceOne = false;
        for (int i = 0; i < ConnectedCCDList.size(); i++)
        {
            // 修复：检查索引是否有效
            if (ConnectedCCDList[i] >= 0 && ConnectedCCDList[i] < indi_Client->GetDeviceCount()) {
                INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedCCDList[i]);
                if (device != nullptr) {
                    emit wsThread->sendMessageToClient("DeviceToBeAllocated:CCD:" + QString::number(ConnectedCCDList[i]) + ":" + QString::fromUtf8(device->getDeviceName())); // already allocated
                }
            }
        }
    }

    if (ConnectedTELESCOPEList.size() == 1)
    {
        Logger::Log("Mount Connected Success and Mount device is only one!", LogLevel::INFO, DeviceType::MAIN);
        // 修复：检查向量和索引有效性
        if (!ConnectedTELESCOPEList.empty() && ConnectedTELESCOPEList[0] >= 0 && ConnectedTELESCOPEList[0] < indi_Client->GetDeviceCount()) {
            INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedTELESCOPEList[0]);
            if (device != nullptr) {
                dpMount = device;
                if (systemdevicelist.system_devices.size() > 0) {
                    systemdevicelist.system_devices[0].isConnect = true;
                }
                AfterDeviceConnect(dpMount);
            }
        }
    }
    else if (ConnectedTELESCOPEList.size() > 1)
    {
        EachDeviceOne = false;
        for (int i = 0; i < ConnectedTELESCOPEList.size(); i++)
        {
            // 修复：检查索引有效性
            if (ConnectedTELESCOPEList[i] >= 0 && ConnectedTELESCOPEList[i] < indi_Client->GetDeviceCount()) {
                INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedTELESCOPEList[i]);
                if (device != nullptr) {
                    emit wsThread->sendMessageToClient("DeviceToBeAllocated:Mount:" + QString::number(ConnectedTELESCOPEList[i]) + ":" + QString::fromUtf8(device->getDeviceName()));
                }
            }
        }
    }

    if (ConnectedFOCUSERList.size() == 1)
    {
        Logger::Log("Focuser Connected Success and Focuser device is only one!", LogLevel::INFO, DeviceType::MAIN);
        // 修复：检查向量和索引有效性
        if (!ConnectedFOCUSERList.empty() && ConnectedFOCUSERList[0] >= 0 && ConnectedFOCUSERList[0] < indi_Client->GetDeviceCount()) {
            INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedFOCUSERList[0]);
            if (device != nullptr) {
                dpFocuser = device;
                if (systemdevicelist.system_devices.size() > 22) {
                    systemdevicelist.system_devices[22].isConnect = true;
                }
                AfterDeviceConnect(dpFocuser);
            }
        }
    }
    else if (ConnectedFOCUSERList.size() > 1)
    {
        EachDeviceOne = false;
        for (int i = 0; i < ConnectedFOCUSERList.size(); i++)
        {
            // 修复：检查索引有效性
            if (ConnectedFOCUSERList[i] >= 0 && ConnectedFOCUSERList[i] < indi_Client->GetDeviceCount()) {
                INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedFOCUSERList[i]);
                if (device != nullptr) {
                    emit wsThread->sendMessageToClient("DeviceToBeAllocated:Focuser:" + QString::number(ConnectedFOCUSERList[i]) + ":" + QString::fromUtf8(device->getDeviceName()));
                }
            }
        }
    }

    if (ConnectedFILTERList.size() == 1)
    {
        Logger::Log("Filter Connected Success and Filter device is only one!", LogLevel::INFO, DeviceType::MAIN);
        // 修复：检查向量和索引有效性
        if (!ConnectedFILTERList.empty() && ConnectedFILTERList[0] >= 0 && ConnectedFILTERList[0] < indi_Client->GetDeviceCount()) {
            INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedFILTERList[0]);
            if (device != nullptr) {
                dpCFW = device;
                if (systemdevicelist.system_devices.size() > 21) {
                    systemdevicelist.system_devices[21].isConnect = true;
                }
                AfterDeviceConnect(dpCFW);
            }
        }
    }
    else if (ConnectedFILTERList.size() > 1)
    {
        EachDeviceOne = false;
        for (int i = 0; i < ConnectedFILTERList.size(); i++)
        {
            // 修复：检查索引有效性
            if (ConnectedFILTERList[i] >= 0 && ConnectedFILTERList[i] < indi_Client->GetDeviceCount()) {
                INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedFILTERList[i]);
                if (device != nullptr) {
                    emit wsThread->sendMessageToClient("DeviceToBeAllocated:CFW:" + QString::number(ConnectedFILTERList[i]) + ":" + QString::fromUtf8(device->getDeviceName()));
                }
            }
        }
    }

    Logger::Log("Each Device Only Has One:" + std::to_string(EachDeviceOne), LogLevel::INFO, DeviceType::MAIN);
    if (EachDeviceOne)
    {
        // AfterDeviceConnect();
    }
    else
    {
        emit wsThread->sendMessageToClient("ShowDeviceAllocationWindow");
    }
}

void MainWindow::BindingDevice(QString DeviceType, int DeviceIndex)
{
    indi_Client->PrintDevices();
    Logger::Log("BindingDevice:" + DeviceType.toStdString() + ":" + QString::number(DeviceIndex).toStdString(), LogLevel::INFO, DeviceType::MAIN);

    // 修复：检查DeviceIndex是否有效
    if (DeviceIndex < 0 || DeviceIndex >= indi_Client->GetDeviceCount()) {
        Logger::Log("BindingDevice | Invalid DeviceIndex: " + std::to_string(DeviceIndex), LogLevel::ERROR, DeviceType::MAIN);
        return;
    }
    
    INDI::BaseDevice *device = indi_Client->GetDeviceFromList(DeviceIndex);
    if (device == nullptr) {
        Logger::Log("BindingDevice | GetDeviceFromList returned nullptr for DeviceIndex: " + std::to_string(DeviceIndex), LogLevel::ERROR, DeviceType::MAIN);
        return;
    }
    
    if (DeviceType == "Guider")
    {
        Logger::Log("Binding Guider Device start ...", LogLevel::INFO, DeviceType::MAIN);
        dpGuider = device;
        indi_Client->disconnectDevice(device->getDeviceName());
        Logger::Log("Disconnect Guider Device", LogLevel::INFO, DeviceType::MAIN);
        sleep(1);
        call_phd_whichCamera(device->getDeviceName());
        sleep(2);
        Logger::Log("Call PHD2 Guider Connect", LogLevel::INFO, DeviceType::MAIN);
        if (systemdevicelist.system_devices.size() > 1) {
            systemdevicelist.system_devices[1].isConnect = true;
            systemdevicelist.system_devices[1].isBind = true;
        }
        AfterDeviceConnect(dpGuider);
        Logger::Log("Binding Guider Device end !", LogLevel::INFO, DeviceType::MAIN);
    }
    else if (DeviceType == "MainCamera")
    {
        Logger::Log("Binding MainCamera Device start ...", LogLevel::INFO, DeviceType::MAIN);
        dpMainCamera = device;
        if (systemdevicelist.system_devices.size() > 20) {
            systemdevicelist.system_devices[20].isConnect = true;
            systemdevicelist.system_devices[20].isBind = true;
        }
        AfterDeviceConnect(dpMainCamera);
        Logger::Log("Binding MainCamera Device end !", LogLevel::INFO, DeviceType::MAIN);
    }
    else if (DeviceType == "Mount")
    {
        Logger::Log("Binding Mount Device start ...", LogLevel::INFO, DeviceType::MAIN);
        dpMount = device;
        if (systemdevicelist.system_devices.size() > 0) {
            systemdevicelist.system_devices[0].isConnect = true;
            systemdevicelist.system_devices[0].isBind = true;
        }
        AfterDeviceConnect(dpMount);
        Logger::Log("Binding Mount Device end !", LogLevel::INFO, DeviceType::MAIN);
    }
    else if (DeviceType == "Focuser")
    {
        Logger::Log("Binding Focuser Device start ...", LogLevel::INFO, DeviceType::MAIN);
        dpFocuser = device;
        if (systemdevicelist.system_devices.size() > 22) {
            systemdevicelist.system_devices[22].isConnect = true;
            systemdevicelist.system_devices[22].isBind = true;
        }
        AfterDeviceConnect(dpFocuser);
        Logger::Log("Binding Focuser Device end !", LogLevel::INFO, DeviceType::MAIN);
    }
    else if (DeviceType == "PoleCamera")
    {
        Logger::Log("Binding PoleCamera Device start ...", LogLevel::INFO, DeviceType::MAIN);
        // 修复：使用已检查的device指针
        dpPoleScope = device;
        if (systemdevicelist.system_devices.size() > 2) {
            systemdevicelist.system_devices[2].isConnect = true;
            systemdevicelist.system_devices[2].isBind = true;
        }
        AfterDeviceConnect(dpPoleScope);
        Logger::Log("Binding PoleCamera Device end !", LogLevel::INFO, DeviceType::MAIN);
    }
    else if (DeviceType == "CFW")
    {
        Logger::Log("Binding CFW Device start ...", LogLevel::INFO, DeviceType::MAIN);
        dpCFW = indi_Client->GetDeviceFromList(DeviceIndex);
        systemdevicelist.system_devices[21].isConnect = true;
        systemdevicelist.system_devices[21].isBind = true;
        AfterDeviceConnect(dpCFW);
        Logger::Log("Binding CFW Device end !", LogLevel::INFO, DeviceType::MAIN);
    }
}
void MainWindow::UnBindingDevice(QString DeviceType)
{
    indi_Client->PrintDevices();
    Logger::Log("UnBindingDevice:" + DeviceType.toStdString(), LogLevel::INFO, DeviceType::MAIN);

    if (DeviceType == "Guider")
    {
        Logger::Log("UnBinding Guider Device start ...", LogLevel::INFO, DeviceType::MAIN);
        call_phd_StopLooping();
        isGuiding = false;
        emit wsThread->sendMessageToClient("GuiderSwitchStatus:false");
        isGuiderLoopExp = false;
        emit wsThread->sendMessageToClient("GuiderLoopExpStatus:false");
        emit wsThread->sendMessageToClient("GuiderUpdateStatus:0");

        indi_Client->disconnectDevice(dpGuider->getDeviceName());
        Logger::Log("Disconnect Guider Device", LogLevel::INFO, DeviceType::MAIN);
        sleep(1);
        indi_Client->setBaudRate(dpGuider, systemdevicelist.system_devices[1].BaudRate);
        indi_Client->connectDevice(dpGuider->getDeviceName());
        Logger::Log("Connect Guider Device", LogLevel::INFO, DeviceType::MAIN);
        sleep(3);
        int DeviceIndex;
        for (int i = 0; i < indi_Client->GetDeviceCount(); i++) //  indi_Client->GetDeviceFromList(i)
        {
            if (indi_Client->GetDeviceFromList(i)->getDeviceName() == dpGuider->getDeviceName())
            {
                DeviceIndex = i;
            }
        }
        systemdevicelist.system_devices[1].isBind = false;
        systemdevicelist.system_devices[1].DeviceIndiName = "";
        dpGuider = nullptr;
        Logger::Log("UnBinding Guider Device end !", LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("DeviceToBeAllocated:CCD:" + QString::number(DeviceIndex) + ":" + QString::fromUtf8(indi_Client->GetDeviceFromList(DeviceIndex)->getDeviceName()));
    }
    else if (DeviceType == "MainCamera")
    {
        Logger::Log("UnBinding MainCamera Device(" + std::string(dpMainCamera->getDeviceName()) + ") start ...", LogLevel::INFO, DeviceType::MAIN);

        int DeviceIndex;
        for (int i = 0; i < indi_Client->GetDeviceCount(); i++) //  indi_Client->GetDeviceFromList(i)
        {
            if (indi_Client->GetDeviceFromList(i)->getDeviceName() == dpMainCamera->getDeviceName())
            {
                DeviceIndex = i;
            }
        }

        QString CFWname;
        indi_Client->getCFWSlotName(dpMainCamera, CFWname);
        if (CFWname != "")
        {
            emit wsThread->sendMessageToClient("deleteDeviceTypeAllocationList:CFW");
        }
        systemdevicelist.system_devices[20].isBind = false;
        systemdevicelist.system_devices[20].DeviceIndiName = "";
        dpMainCamera = nullptr;

        emit wsThread->sendMessageToClient("DeviceToBeAllocated:CCD:" + QString::number(DeviceIndex) + ":" + QString::fromUtf8(indi_Client->GetDeviceFromList(DeviceIndex)->getDeviceName())); // already allocated
    }
    else if (DeviceType == "Mount")
    {
        int DeviceIndex;
        for (int i = 0; i < indi_Client->GetDeviceCount(); i++) //  indi_Client->GetDeviceFromList(i)
        {
            if (indi_Client->GetDeviceFromList(i)->getDeviceName() == dpMount->getDeviceName())
            {
                DeviceIndex = i;
            }
        }
        systemdevicelist.system_devices[0].isBind = false;
        systemdevicelist.system_devices[0].DeviceIndiName = "";
        dpMount = nullptr;

        emit wsThread->sendMessageToClient("DeviceToBeAllocated:Mount:" + QString::number(DeviceIndex) + ":" + QString::fromUtf8(indi_Client->GetDeviceFromList(DeviceIndex)->getDeviceName()));
    }
    else if (DeviceType == "Focuser")
    {
        int DeviceIndex;
        for (int i = 0; i < indi_Client->GetDeviceCount(); i++) //  indi_Client->GetDeviceFromList(i)
        {
            if (indi_Client->GetDeviceFromList(i)->getDeviceName() == dpFocuser->getDeviceName())
            {
                DeviceIndex = i;
            }
        }
        systemdevicelist.system_devices[22].isBind = false;
        systemdevicelist.system_devices[22].DeviceIndiName = "";
        dpFocuser = nullptr;

        emit wsThread->sendMessageToClient("DeviceToBeAllocated:Focuser:" + QString::number(DeviceIndex) + ":" + QString::fromUtf8(indi_Client->GetDeviceFromList(DeviceIndex)->getDeviceName()));
    }
    else if (DeviceType == "PoleCamera")
    {
        int DeviceIndex;
        for (int i = 0; i < indi_Client->GetDeviceCount(); i++) //  indi_Client->GetDeviceFromList(i)
        {
            if (indi_Client->GetDeviceFromList(i)->getDeviceName() == dpPoleScope->getDeviceName())
            {
                DeviceIndex = i;
            }
        }
        systemdevicelist.system_devices[2].isBind = false;
        systemdevicelist.system_devices[2].DeviceIndiName = "";
        dpPoleScope = nullptr;

        emit wsThread->sendMessageToClient("DeviceToBeAllocated:CCD:" + QString::number(DeviceIndex) + ":" + QString::fromUtf8(indi_Client->GetDeviceFromList(DeviceIndex)->getDeviceName()));
    }
    else if (DeviceType == "CFW")
    {
        int DeviceIndex;
        for (int i = 0; i < indi_Client->GetDeviceCount(); i++) //  indi_Client->GetDeviceFromList(i)
        {
            if (indi_Client->GetDeviceFromList(i)->getDeviceName() == dpCFW->getDeviceName())
            {
                DeviceIndex = i;
            }
        }
        systemdevicelist.system_devices[21].isBind = false;
        systemdevicelist.system_devices[21].DeviceIndiName = "";
        dpCFW = nullptr;

        emit wsThread->sendMessageToClient("DeviceToBeAllocated:CFW:" + QString::number(DeviceIndex) + ":" + QString::fromUtf8(indi_Client->GetDeviceFromList(DeviceIndex)->getDeviceName()));
    }

    indi_Client->PrintDevices();
}

void MainWindow::AfterDeviceConnect()
{
    Logger::Log("Starting AfterDeviceConnect process.", LogLevel::INFO, DeviceType::MAIN);

    if (dpMainCamera != NULL)
    {
        if (isDSLR(dpMainCamera) && NotSetDSLRsInfo)
        {
            QString CameraName = dpMainCamera->getDeviceName();
            Logger::Log("This may be a DSLRs Camera, need to set Resolution and pixel size. Camera: " + CameraName.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
            DSLRsInfo DSLRsInfo = Tools::readDSLRsInfo(CameraName);
            if (DSLRsInfo.Name == CameraName && DSLRsInfo.SizeX != 0 && DSLRsInfo.SizeY != 0 && DSLRsInfo.PixelSize != 0)
            {
                indi_Client->setCCDBasicInfo(dpMainCamera, DSLRsInfo.SizeX, DSLRsInfo.SizeY, DSLRsInfo.PixelSize, DSLRsInfo.PixelSize, DSLRsInfo.PixelSize, 8);
                Logger::Log("Updated CCD Basic Info for DSLRs Camera.", LogLevel::INFO, DeviceType::MAIN);
                emit wsThread->sendMessageToClient("DSLRsSetup:" + QString::fromUtf8(dpMainCamera->getDeviceName()) + ":" + QString::number(DSLRsInfo.SizeX) + ":" + QString::number(DSLRsInfo.SizeY) + ":" + QString::number(DSLRsInfo.PixelSize));
                return;
            }
            else
            {
                emit wsThread->sendMessageToClient("DSLRsSetup:" + QString::fromUtf8(dpMainCamera->getDeviceName()));
                return;
            }
        }
        NotSetDSLRsInfo = true;

        if (isDSLR(dpMainCamera) ){
            indi_Client->disableDSLRLiveView(dpMainCamera);
            Logger::Log("Disabled DSLR Live View for Camera: " + QString::fromUtf8(dpMainCamera->getDeviceName()).toStdString(), LogLevel::INFO, DeviceType::MAIN);
        }

        indi_Client->GetAllPropertyName(dpMainCamera);
        Logger::Log("MainCamera connected after Device(" + QString::fromUtf8(dpMainCamera->getDeviceName()).toStdString() + ") Connect: " + QString::fromUtf8(dpMainCamera->getDeviceName()).toStdString(), LogLevel::INFO, DeviceType::MAIN);

        ConnectedDevices.push_back({"MainCamera", QString::fromUtf8(dpMainCamera->getDeviceName())});

        systemdevicelist.system_devices[20].DeviceIndiName = QString::fromUtf8(dpMainCamera->getDeviceName());
        systemdevicelist.system_devices[20].isBind = true;

        indi_Client->setBLOBMode(B_ALSO, dpMainCamera->getDeviceName(), nullptr);
        indi_Client->enableDirectBlobAccess(dpMainCamera->getDeviceName(), nullptr);

        QString SDKVERSION;
        indi_Client->getCCDSDKVersion(dpMainCamera, SDKVERSION);
        emit wsThread->sendMessageToClient("getSDKVersion:MainCamera:" + SDKVERSION);
        Logger::Log("MainCamera SDK version: " + SDKVERSION.toStdString(), LogLevel::INFO, DeviceType::MAIN);

        indi_Client->getCCDOffset(dpMainCamera, glOffsetValue, glOffsetMin, glOffsetMax);
        Logger::Log("CCD Offset - Value: " + std::to_string(glOffsetValue) + ", Min: " + std::to_string(glOffsetMin) + ", Max: " + std::to_string(glOffsetMax), LogLevel::INFO, DeviceType::MAIN);

        indi_Client->getCCDGain(dpMainCamera, glGainValue, glGainMin, glGainMax);
        Logger::Log("CCD Gain - Value: " + std::to_string(glGainValue) + ", Min: " + std::to_string(glGainMin) + ", Max: " + std::to_string(glGainMax), LogLevel::INFO, DeviceType::MAIN);

        int maxX, maxY;
        double pixelsize, pixelsizX, pixelsizY;
        int bitDepth;
        indi_Client->getCCDBasicInfo(dpMainCamera, maxX, maxY, pixelsize, pixelsizX, pixelsizY, bitDepth);
        Logger::Log("CCD Basic Info - MaxX: " + std::to_string(maxX) + ", MaxY: " + std::to_string(maxY) + ", PixelSize: " + std::to_string(pixelsize), LogLevel::INFO, DeviceType::MAIN);

        if (bitDepth != 16)
        {
            Logger::Log("The current camera outputs is not 16-bit data; attempting to modify it to 16-bit.", LogLevel::INFO, DeviceType::CAMERA);
            // indi_Client->setCCDBasicInfo(dpMainCamera, maxX, maxY, pixelsize, pixelsizX, pixelsizY, 16);
        }

        indi_Client->getCCDBasicInfo(dpMainCamera, maxX, maxY, pixelsize, pixelsizX, pixelsizY, bitDepth);
        if (bitDepth != 16)
        {
            Logger::Log("Failed to set the camera bit depth to 16-bit.", LogLevel::WARNING, DeviceType::CAMERA);
        }

        // 设置初始温度
        indi_Client->setTemperature(dpMainCamera, CameraTemperature);
        Logger::Log("CCD Temperature set to: " + std::to_string(CameraTemperature), LogLevel::INFO, DeviceType::MAIN);

        glCameraSize_width = maxX * pixelsize / 1000;
        glCameraSize_width = std::round(glCameraSize_width * 10) / 10;
        glCameraSize_height = maxY * pixelsize / 1000;
        glCameraSize_height = std::round(glCameraSize_height * 10) / 10;
        Logger::Log("CCD Chip size - Width: " + std::to_string(glCameraSize_width) + ", Height: " + std::to_string(glCameraSize_height), LogLevel::INFO, DeviceType::MAIN);

        int X, Y;
        indi_Client->getCCDFrameInfo(dpMainCamera, X, Y, glMainCCDSizeX, glMainCCDSizeY);
        Logger::Log("CCD Frame Info - SizeX: " + std::to_string(glMainCCDSizeX) + ", SizeY: " + std::to_string(glMainCCDSizeY), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("MainCameraSize:" + QString::number(glMainCCDSizeX) + ":" + QString::number(glMainCCDSizeY));

        int offsetX, offsetY;
        indi_Client->getCCDCFA(dpMainCamera, offsetX, offsetY, MainCameraCFA);
        Logger::Log("CCD CFA Info - OffsetX: " + std::to_string(offsetX) + ", OffsetY: " + std::to_string(offsetY) + ", CFA: " + MainCameraCFA.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("MainCameraCFA:" + MainCameraCFA);
        indi_Client->setCCDUploadModeToLacal(dpMainCamera);
        indi_Client->setCCDUpload(dpMainCamera, "/dev/shm", "ccd_simulator");

        QString CFWname;
        indi_Client->getCFWSlotName(dpMainCamera, CFWname);
        if (CFWname != "")
        {
            Logger::Log("CFW Slot Name: " + CFWname.toStdString(), LogLevel::INFO, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("ConnectSuccess:CFW:" + CFWname + " (on camera):" + QString::fromUtf8(dpMainCamera->getDriverExec()));
            isFilterOnCamera = true;

            int min, max, pos;
            indi_Client->getCFWPosition(dpMainCamera, pos, min, max);
            Logger::Log("CFW Position - Min: " + std::to_string(min) + ", Max: " + std::to_string(max) + ", Current: " + std::to_string(pos), LogLevel::INFO, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("CFWPositionMax:" + QString::number(max));
            Logger::Log("CFW connected successfully.", LogLevel::INFO, DeviceType::MAIN);
        }
        Logger::Log("MainCamera connected successfully.", LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConnectSuccess:MainCamera:" + QString::fromUtf8(dpMainCamera->getDeviceName()) + ":" + QString::fromUtf8(dpMainCamera->getDriverExec()));
    }

    if (dpMount != NULL)
    {
        Logger::Log("Mount connected after Device(" + QString::fromUtf8(dpMount->getDeviceName()).toStdString() + ") Connect: " + QString::fromUtf8(dpMount->getDeviceName()).toStdString(), LogLevel::INFO, DeviceType::MAIN);
        
        ConnectedDevices.push_back({"Mount", QString::fromUtf8(dpMount->getDeviceName())});

        systemdevicelist.system_devices[0].DeviceIndiName = QString::fromUtf8(dpMount->getDeviceName());
        systemdevicelist.system_devices[0].isBind = true;

        indi_Client->GetAllPropertyName(dpMount);
        QString DevicePort;
        indi_Client->getDevicePort(dpMount, DevicePort);
        emit wsThread->sendMessageToClient("getDevicePort:Mount:" + DevicePort);
        Logger::Log("Device port for Mount: " + DevicePort.toStdString(), LogLevel::INFO, DeviceType::MAIN);

        getClientSettings();
        getMountParameters();
        indi_Client->setLocation(dpMount, observatorylatitude, observatorylongitude, 50);
        indi_Client->setAutoFlip(dpMount, false);
        indi_Client->setMinutesPastMeridian(dpMount, 1, -1);

        indi_Client->setAUXENCODERS(dpMount);

        QDateTime datetime = QDateTime::currentDateTime();
        indi_Client->setTimeUTC(dpMount, datetime);
        Logger::Log("UTC Time set for Mount: " + datetime.toString(Qt::ISODate).toStdString(), LogLevel::INFO, DeviceType::MAIN);
        indi_Client->getTimeUTC(dpMount, datetime);
        Logger::Log("UTC Time: " + datetime.currentDateTimeUtc().toString(Qt::ISODate).toStdString(), LogLevel::INFO, DeviceType::MAIN);

        double a, b, c, d;
        indi_Client->getTelescopeInfo(dpMount, a, b, c, d);
        Logger::Log("Telescope Info - A: " + std::to_string(a) + ", B: " + std::to_string(b) + ", C: " + std::to_string(c) + ", D: " + std::to_string(d), LogLevel::INFO, DeviceType::MAIN);

        indi_Client->getTelescopeRADECJ2000(dpMount, a, b);
        indi_Client->getTelescopeRADECJNOW(dpMount, a, b);

        bool isPark;
        indi_Client->getTelescopePark(dpMount, isPark);
        Logger::Log("Telescope Park Status: " + std::string(isPark ? "Parked" : "Not Parked"), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("TelescopePark:" + QString::fromStdString(isPark ? "ON" : "OFF"));

        int maxspeed, minspeed, speedvalue, total;
        indi_Client->getTelescopeTotalSlewRate(dpMount, total);
        glTelescopeTotalSlewRate = total;
        Logger::Log("Telescope Total Slew Rate: " + std::to_string(total), LogLevel::INFO, DeviceType::MAIN);

        emit wsThread->sendMessageToClient("TelescopeTotalSlewRate:" + QString::number(total));
        indi_Client->getTelescopeMaxSlewRateOptions(dpMount, minspeed, maxspeed, speedvalue);
        indi_Client->setTelescopeSlewRate(dpMount, total);
        int speed;
        indi_Client->getTelescopeSlewRate(dpMount, speed);
        Logger::Log("Current Telescope Slew Rate: " + std::to_string(speed), LogLevel::INFO, DeviceType::MAIN);
        // emit wsThread->sendMessageToClient("TelescopeCurrentSlewRate:" + QString::number(speed));
        emit wsThread->sendMessageToClient("MountSetSpeedSuccess:" + QString::number(speed));
        indi_Client->setTelescopeTrackEnable(dpMount, true);

        bool isTrack = false;
        indi_Client->getTelescopeTrackEnable(dpMount, isTrack);

        if (isTrack)
        {
            emit wsThread->sendMessageToClient("TelescopeTrack:ON");
        }
        else
        {
            emit wsThread->sendMessageToClient("TelescopeTrack:OFF");
        }
        Logger::Log("Telescope Tracking Status: " + std::string(isTrack ? "Enabled" : "Disabled"), LogLevel::INFO, DeviceType::MAIN);

        indi_Client->setTelescopeTrackRate(dpMount, "SIDEREAL");
        QString side;
        indi_Client->getTelescopePierSide(dpMount, side);
        Logger::Log("Telescope Pier Side: " + side.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("TelescopePierSide:" + side);
        Logger::Log("Mount connected successfully.", LogLevel::INFO, DeviceType::MAIN);

        // 获取驱动版本号
        QString MountSDKVersion = "null";
        indi_Client->getMountInfo(dpMount, MountSDKVersion);
        emit wsThread->sendMessageToClient("getMountInfo:" + MountSDKVersion);
        Logger::Log("Mount Info: " + MountSDKVersion.toStdString(), LogLevel::INFO, DeviceType::MAIN);

        // indi_Client->setTelescopeHomeInit(dpMount, "SYNCHOME");
        indi_Client->mountState.updateHomeRAHours(observatorylatitude, observatorylongitude);
        indi_Client->mountState.printCurrentState();
        emit wsThread->sendMessageToClient("ConnectSuccess:Mount:" + QString::fromUtf8(dpMount->getDeviceName()) + ":" + QString::fromUtf8(dpMount->getDriverExec()));
    }

    if (dpFocuser != NULL)
    {
        Logger::Log("Focuser connected after Device(" + QString::fromUtf8(dpFocuser->getDeviceName()).toStdString() + ") Connect: " + dpFocuser->getDeviceName(), LogLevel::INFO, DeviceType::MAIN);
        
        ConnectedDevices.push_back({"Focuser", QString::fromUtf8(dpFocuser->getDeviceName())});

        systemdevicelist.system_devices[22].DeviceIndiName = QString::fromUtf8(dpFocuser->getDeviceName());
        systemdevicelist.system_devices[22].isBind = true;

        indi_Client->GetAllPropertyName(dpFocuser);
        // indi_Client->syncFocuserPosition(dpFocuser, 0);
        CurrentPosition = FocuserControl_getPosition();

        // 获取焦点器最大和最小位置
        int min, max, value, step;
        indi_Client->getFocuserRange(dpFocuser, min, max, step, value);
        // Logger::Log("Focuser Range - Min: " + std::to_string(min) + ", Max: " + std::to_string(max) + ", Value: " + std::to_string(value) + ", Step: " + std::to_string(step), LogLevel::INFO, DeviceType::MAIN);
        // focuserMaxPosition = std::min(max, focuserMaxPosition);
        // focuserMinPosition = std::max(min, focuserMinPosition);
        getFocuserParameters();
        QString SDKVERSION = "null";
        indi_Client->getFocuserSDKVersion(dpFocuser, SDKVERSION);
        emit wsThread->sendMessageToClient("getSDKVersion:Focuser:" + SDKVERSION);
        Logger::Log("Focuser SDK version: " + SDKVERSION.toStdString(), LogLevel::INFO, DeviceType::MAIN);

        QString DevicePort;
        indi_Client->getDevicePort(dpFocuser, DevicePort);
        emit wsThread->sendMessageToClient("getDevicePort:Focuser:" + DevicePort);
        Logger::Log("Device port for Focuser: " + DevicePort.toStdString(), LogLevel::INFO, DeviceType::MAIN);

        if (focuserMaxPosition == -1 && focuserMinPosition == -1)
        {
            focuserMaxPosition = max;
            focuserMinPosition = min;
            Tools::saveParameter("Focuser", "focuserMaxPosition", QString::number(focuserMaxPosition));
            Tools::saveParameter("Focuser", "focuserMinPosition", QString::number(focuserMinPosition));
        }
        emit wsThread->sendMessageToClient("FocuserLimit:" + QString::number(focuserMinPosition) + ":" + QString::number(focuserMaxPosition));
        Logger::Log("Focuser Max Position: " + std::to_string(focuserMaxPosition) + ", Min Position: " + std::to_string(focuserMinPosition), LogLevel::INFO, DeviceType::MAIN);
        Logger::Log("Focuser Current Position: " + std::to_string(CurrentPosition), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("FocusPosition:" + QString::number(CurrentPosition) + ":" + QString::number(CurrentPosition));
        Logger::Log("Focuser connected successfully.", LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConnectSuccess:Focuser:" + QString::fromUtf8(dpFocuser->getDeviceName()) + ":" + QString::fromUtf8(dpFocuser->getDriverExec()));
    }

    if (dpCFW != NULL)
    {
        Logger::Log("CFW connected after Device(" + QString::fromUtf8(dpCFW->getDeviceName()).toStdString() + ") Connect: " + dpCFW->getDeviceName(), LogLevel::INFO, DeviceType::MAIN);
        
        ConnectedDevices.push_back({"CFW", QString::fromUtf8(dpCFW->getDeviceName())});

        systemdevicelist.system_devices[21].DeviceIndiName = QString::fromUtf8(dpCFW->getDeviceName());
        systemdevicelist.system_devices[21].isBind = true;

        indi_Client->GetAllPropertyName(dpCFW);
        int min, max, pos;
        indi_Client->getCFWPosition(dpCFW, pos, min, max);
        Logger::Log("CFW Position - Min: " + std::to_string(min) + ", Max: " + std::to_string(max) + ", Current: " + std::to_string(pos), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("CFWPositionMax:" + QString::number(max));
        Logger::Log("CFW connected successfully.", LogLevel::INFO, DeviceType::MAIN);
        if (Tools::readCFWList(QString::fromUtf8(dpCFW->getDeviceName())) != QString())
        {
            emit wsThread->sendMessageToClient("getCFWList:" + Tools::readCFWList(QString::fromUtf8(dpCFW->getDeviceName())));
        }
        emit wsThread->sendMessageToClient("ConnectSuccess:CFW:" + QString::fromUtf8(dpCFW->getDeviceName()) + ":" + QString::fromUtf8(dpCFW->getDriverExec()));
    }

    if (dpGuider != NULL)
    {
        Logger::Log("Guider connected after Device(" + QString::fromUtf8(dpGuider->getDeviceName()).toStdString() + ") Connect: " + dpGuider->getDeviceName(), LogLevel::INFO, DeviceType::MAIN);
        
        ConnectedDevices.push_back({"Guider", QString::fromUtf8(dpGuider->getDeviceName())});
        Logger::Log("Guider connected successfully.", LogLevel::INFO, DeviceType::MAIN);

        systemdevicelist.system_devices[1].DeviceIndiName = QString::fromUtf8(dpGuider->getDeviceName());
        systemdevicelist.system_devices[1].isBind = true;
        QString SDKVERSION;
        indi_Client->getCCDSDKVersion(dpGuider, SDKVERSION);
        emit wsThread->sendMessageToClient("getSDKVersion:Guider:" + SDKVERSION);
        Logger::Log("Guider SDK version: " + SDKVERSION.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConnectSuccess:Guider:" + QString::fromUtf8(dpGuider->getDeviceName()) + ":" + QString::fromUtf8(dpGuider->getDriverExec()));
    }
    Logger::Log("All devices connected after successfully.", LogLevel::INFO, DeviceType::MAIN);
}
void MainWindow::AfterDeviceConnect(INDI::BaseDevice *dp)
{
    if (dpMainCamera == dp)
    {
        if (isDSLR(dpMainCamera) && NotSetDSLRsInfo)
        {
            QString CameraName = dpMainCamera->getDeviceName();
            Logger::Log("This may be a DSLRs Camera, need to set Resolution and pixel size. Camera: " + CameraName.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
            DSLRsInfo DSLRsInfo = Tools::readDSLRsInfo(CameraName);
            if (DSLRsInfo.Name == CameraName && DSLRsInfo.SizeX != 0 && DSLRsInfo.SizeY != 0 && DSLRsInfo.PixelSize != 0)
            {
                indi_Client->setCCDBasicInfo(dpMainCamera, DSLRsInfo.SizeX, DSLRsInfo.SizeY, DSLRsInfo.PixelSize, DSLRsInfo.PixelSize, DSLRsInfo.PixelSize, 8);
                Logger::Log("Updated CCD Basic Info for DSLRs Camera.", LogLevel::INFO, DeviceType::MAIN);
                emit wsThread->sendMessageToClient("DSLRsSetup:" + QString::fromUtf8(dpMainCamera->getDeviceName()) + ":" + QString::number(DSLRsInfo.SizeX) + ":" + QString::number(DSLRsInfo.SizeY) + ":" + QString::number(DSLRsInfo.PixelSize));
                return;
            }
            else
            {
                emit wsThread->sendMessageToClient("DSLRsSetup:" + QString::fromUtf8(dpMainCamera->getDeviceName()));
                return;
            }
        }
        if (isDSLR(dpMainCamera) ){
            indi_Client->disableDSLRLiveView(dpMainCamera);
            Logger::Log("Disabled DSLR Live View for Camera: " + QString::fromUtf8(dpMainCamera->getDeviceName()).toStdString(), LogLevel::INFO, DeviceType::MAIN);
        }

        // 预先获取SDK的值为默认值
        indi_Client->getCCDOffset(dpMainCamera, glOffsetValue, glOffsetMin, glOffsetMax);
        emit wsThread->sendMessageToClient("MainCameraOffsetRange:" + QString::number(glOffsetMin) + ":" + QString::number(glOffsetMax) + ":" + QString::number(glOffsetValue));
        Logger::Log("CCD Offset - Value: " + std::to_string(glOffsetValue) + ", Min: " + std::to_string(glOffsetMin) + ", Max: " + std::to_string(glOffsetMax), LogLevel::INFO, DeviceType::MAIN);

        indi_Client->getCCDGain(dpMainCamera, glGainValue, glGainMin, glGainMax);
        Logger::Log("CCD Gain - Value: " + std::to_string(glGainValue) + ", Min: " + std::to_string(glGainMin) + ", Max: " + std::to_string(glGainMax), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("MainCameraGainRange:" + QString::number(glGainMin) + ":" + QString::number(glGainMax) + ":" + QString::number(glGainValue));

        // 获取主相机所有参数
        getMainCameraParameters();
        NotSetDSLRsInfo = true;
        sleep(1); // 给与初始化数据更新时间
        indi_Client->GetAllPropertyName(dpMainCamera);
        Logger::Log("MainCamera connected after Device(" + QString::fromUtf8(dpMainCamera->getDeviceName()).toStdString() + ") Connect: " + QString::fromUtf8(dpMainCamera->getDeviceName()).toStdString(), LogLevel::INFO, DeviceType::MAIN);
        
        ConnectedDevices.push_back({"MainCamera", QString::fromUtf8(dpMainCamera->getDeviceName())});

        systemdevicelist.system_devices[20].DeviceIndiName = QString::fromUtf8(dpMainCamera->getDeviceName());
        systemdevicelist.system_devices[20].isBind = true;

        indi_Client->setBLOBMode(B_ALSO, dpMainCamera->getDeviceName(), nullptr);
        indi_Client->enableDirectBlobAccess(dpMainCamera->getDeviceName(), nullptr);

        QString SDKVERSION;
        indi_Client->getCCDSDKVersion(dpMainCamera, SDKVERSION);
        emit wsThread->sendMessageToClient("getSDKVersion:MainCamera:" + SDKVERSION);
        Logger::Log("MainCamera SDK version: " + SDKVERSION.toStdString(), LogLevel::INFO, DeviceType::MAIN);

        // 设置初始gain
        indi_Client->setCCDGain(dpMainCamera,CameraGain);

        // 设置初始offset
        indi_Client->setCCDOffset(dpMainCamera,ImageOffset);


        int maxX, maxY;
        double pixelsize, pixelsizX, pixelsizY;
        int bitDepth;


        Logger::Log("CCD Basic Info - MaxX: " + std::to_string(maxX) + ", MaxY: " + std::to_string(maxY) + ", PixelSize: " + std::to_string(pixelsize), LogLevel::INFO, DeviceType::MAIN);
        if (bitDepth != 16)
        {
            Logger::Log("The current camera outputs is not 16-bit data; attempting to modify it to 16-bit.", LogLevel::INFO, DeviceType::CAMERA);
            // indi_Client->setCCDBasicInfo(dpMainCamera, maxX, maxY, pixelsize, pixelsizX, pixelsizY, 16);
        }

        // indi_Client->getCCDBasicInfo(dpMainCamera, maxX, maxY, pixelsize, pixelsizX, pixelsizY, bitDepth);
        if (bitDepth != 16)
        {
            Logger::Log("Failed to set the camera bit depth to 16-bit.", LogLevel::WARNING, DeviceType::CAMERA);
        }
        // 设置初始温度
        indi_Client->setTemperature(dpMainCamera, CameraTemperature);
        Logger::Log("CCD Temperature set to: " + std::to_string(CameraTemperature), LogLevel::INFO, DeviceType::MAIN);

        glCameraSize_width = maxX * pixelsize / 1000;
        glCameraSize_width = std::round(glCameraSize_width * 10) / 10;
        glCameraSize_height = maxY * pixelsize / 1000;
        glCameraSize_height = std::round(glCameraSize_height * 10) / 10;
        Logger::Log("CCD Chip size - Width: " + std::to_string(glCameraSize_width) + ", Height: " + std::to_string(glCameraSize_height), LogLevel::INFO, DeviceType::MAIN);

        int X, Y;
        indi_Client->getCCDFrameInfo(dpMainCamera, X, Y, glMainCCDSizeX, glMainCCDSizeY);
        Logger::Log("CCD Frame Info - SizeX: " + std::to_string(glMainCCDSizeX) + ", SizeY: " + std::to_string(glMainCCDSizeY), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("MainCameraSize:" + QString::number(glMainCCDSizeX) + ":" + QString::number(glMainCCDSizeY));

        int offsetX, offsetY;
        indi_Client->getCCDCFA(dpMainCamera, offsetX, offsetY, MainCameraCFA);
        Logger::Log("CCD CFA Info - OffsetX: " + std::to_string(offsetX) + ", OffsetY: " + std::to_string(offsetY) + ", CFA: " + MainCameraCFA.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("MainCameraCFA:" + MainCameraCFA);
        indi_Client->setCCDUploadModeToLacal(dpMainCamera);
        indi_Client->setCCDUpload(dpMainCamera, "/dev/shm", "ccd_simulator");

        // 计算需要的binning以达到548像素以下
        int requiredBinning = 1;
        int currentSize = glMainCCDSizeX;

        // 逐步增加binning直到像素大小小于等于548
        while (currentSize > 1024 && requiredBinning <= 16)
        {
            requiredBinning *= 2;
            currentSize = glMainCCDSizeX / requiredBinning;
        }

        // 限制最大binning为16
        if (requiredBinning > 16)
        {
            requiredBinning = 16;
        }

        glMainCameraBinning = requiredBinning;

        // 记录选择的binning和最终像素大小
        int finalSize = glMainCCDSizeX / requiredBinning;
        qDebug() << "Camera binning selection: Original size =" << glMainCCDSizeX
                 << "Binning =" << requiredBinning << "Final size =" << finalSize;
        emit wsThread->sendMessageToClient("MainCameraBinning:" + QString::number(glMainCameraBinning));

        QString CFWname;
        indi_Client->getCFWSlotName(dpMainCamera, CFWname);
        if (CFWname != "")
        {
            Logger::Log("CFW Slot Name: " + CFWname.toStdString(), LogLevel::INFO, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("ConnectSuccess:CFW:" + CFWname + " (on camera):" + QString::fromUtf8(dpMainCamera->getDriverExec()));
            isFilterOnCamera = true;

            int min, max, pos;
            indi_Client->getCFWPosition(dpMainCamera, pos, min, max);
            Logger::Log("CFW Position - Min: " + std::to_string(min) + ", Max: " + std::to_string(max) + ", Current: " + std::to_string(pos), LogLevel::INFO, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("CFWPositionMax:" + QString::number(max));
        }
        Logger::Log("MainCamera connected successfully.", LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConnectSuccess:MainCamera:" + QString::fromUtf8(dpMainCamera->getDeviceName()) + ":" + QString::fromUtf8(dpMainCamera->getDriverExec()));
    }

    if (dpMount == dp)
    {
        Logger::Log("Mount connected after Device(" + QString::fromUtf8(dpMount->getDeviceName()).toStdString() + ") Connect: " + QString::fromUtf8(dpMount->getDeviceName()).toStdString(), LogLevel::INFO, DeviceType::MAIN);
        
        ConnectedDevices.push_back({"Mount", QString::fromUtf8(dpMount->getDeviceName())});

        systemdevicelist.system_devices[0].DeviceIndiName = QString::fromUtf8(dpMount->getDeviceName());
        systemdevicelist.system_devices[0].isBind = true;
        QString DevicePort;

        indi_Client->GetAllPropertyName(dpMount);

        getClientSettings();
        getMountParameters();
        indi_Client->setLocation(dpMount, observatorylatitude, observatorylongitude, 50);
        Logger::Log("Mount location set to Latitude: " + QString::number(observatorylatitude).toStdString() + ", Longitude: " + QString::number(observatorylongitude).toStdString(), LogLevel::INFO, DeviceType::MAIN);
        indi_Client->setAutoFlip(dpMount, false);
        indi_Client->setMinutesPastMeridian(dpMount, 1, -1);


        indi_Client->setAUXENCODERS(dpMount);


        indi_Client->getDevicePort(dpMount, DevicePort);
        emit wsThread->sendMessageToClient("getDevicePort:Mount:" + DevicePort);
        Logger::Log("Device port for Mount: " + DevicePort.toStdString(), LogLevel::INFO, DeviceType::MAIN);

        // double glLongitude_radian, glLatitude_radian;
        // glLongitude_radian = Tools::getDecAngle(localLat);
        // glLatitude_radian = Tools::getDecAngle(localLon);
        // Logger::Log("Mount location set to Longitude: " + QString::number(Tools::RadToDegree(glLongitude_radian)).toStdString() + ", Latitude: " + QString::number(Tools::RadToDegree(glLatitude_radian)).toStdString(), LogLevel::INFO, DeviceType::MAIN);

        // indi_Client->setLocation(dpMount, Tools::RadToDegree(glLatitude_radian), Tools::RadToDegree(glLongitude_radian), 10);
        QDateTime datetime = QDateTime::currentDateTime();
        indi_Client->setTimeUTC(dpMount, datetime);
        Logger::Log("UTC Time set for Mount: " + datetime.toString(Qt::ISODate).toStdString(), LogLevel::INFO, DeviceType::MAIN);
        indi_Client->getTimeUTC(dpMount, datetime);
        Logger::Log("UTC Time: " + datetime.currentDateTimeUtc().toString(Qt::ISODate).toStdString(), LogLevel::INFO, DeviceType::MAIN);

        double a, b, c, d;
        indi_Client->getTelescopeInfo(dpMount, a, b, c, d);
        Logger::Log("Telescope Info - a: " + std::to_string(a) + ", b: " + std::to_string(b) + ", c: " + std::to_string(c) + ", d: " + std::to_string(d), LogLevel::INFO, DeviceType::MAIN);

        indi_Client->getTelescopeRADECJ2000(dpMount, a, b);
        Logger::Log("Telescope RA/DEC J2000 - RA: " + std::to_string(a) + ", DEC: " + std::to_string(b), LogLevel::INFO, DeviceType::MAIN);
        indi_Client->getTelescopeRADECJNOW(dpMount, a, b);
        Logger::Log("Telescope RA/DEC JNOW - RA: " + std::to_string(a) + ", DEC: " + std::to_string(b), LogLevel::INFO, DeviceType::MAIN);

        bool isPark;
        indi_Client->getTelescopePark(dpMount, isPark);
        Logger::Log("Telescope Park Status: " + std::string(isPark ? "Parked" : "Unparked"), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("TelescopePark:" + QString::fromStdString(isPark ? "ON" : "OFF"));

        int maxspeed, minspeed, speedvalue, total;
        indi_Client->getTelescopeTotalSlewRate(dpMount, total);
        glTelescopeTotalSlewRate = total;
        Logger::Log("Telescope Total Slew Rate: " + std::to_string(total), LogLevel::INFO, DeviceType::MAIN);

        emit wsThread->sendMessageToClient("TelescopeTotalSlewRate:" + QString::number(total));
        indi_Client->getTelescopeMaxSlewRateOptions(dpMount, minspeed, maxspeed, speedvalue);
        indi_Client->setTelescopeSlewRate(dpMount, total);
        int speed;
        indi_Client->getTelescopeSlewRate(dpMount, speed);
        Logger::Log("Current Telescope Slew Rate: " + std::to_string(speed), LogLevel::INFO, DeviceType::MAIN);
        // emit wsThread->sendMessageToClient("TelescopeCurrentSlewRate:" + QString::number(speed));
        emit wsThread->sendMessageToClient("MountSetSpeedSuccess:" + QString::number(speed));
        indi_Client->setTelescopeTrackEnable(dpMount, true);

        bool isTrack = false;
        indi_Client->getTelescopeTrackEnable(dpMount, isTrack);

        if (isTrack)
        {
            emit wsThread->sendMessageToClient("TelescopeTrack:ON");
        }
        else
        {
            emit wsThread->sendMessageToClient("TelescopeTrack:OFF");
        }
        Logger::Log("Telescope Tracking Status: " + std::string(isTrack ? "Enabled" : "Disabled"), LogLevel::INFO, DeviceType::MAIN);
        indi_Client->setTelescopeTrackRate(dpMount, "SIDEREAL");
        QString side;
        indi_Client->getTelescopePierSide(dpMount, side);
        Logger::Log("Telescope Pier Side: " + side.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("TelescopePierSide:" + side);
        Logger::Log("Mount connected successfully.", LogLevel::INFO, DeviceType::MAIN);

        // 获取驱动版本号
        QString MountSDKVersion = "null";
        indi_Client->getMountInfo(dpMount, MountSDKVersion);
        emit wsThread->sendMessageToClient("getMountInfo:" + MountSDKVersion);
        Logger::Log("Mount Info: " + MountSDKVersion.toStdString(), LogLevel::INFO, DeviceType::MAIN);

        // 设置home位置
        // indi_Client->setTelescopeHomeInit(dpMount, "SYNCHOME");
        indi_Client->mountState.updateHomeRAHours(observatorylatitude, observatorylongitude);
        indi_Client->mountState.printCurrentState();
        emit wsThread->sendMessageToClient("ConnectSuccess:Mount:" + QString::fromUtf8(dpMount->getDeviceName()) + ":" + QString::fromUtf8(dpMount->getDriverExec()));
        
    }

    if (dpFocuser == dp)
    {
        Logger::Log("Focuser connected after Device(" + QString::fromUtf8(dpFocuser->getDeviceName()).toStdString() + ") Connect: " + dpFocuser->getDeviceName(), LogLevel::INFO, DeviceType::MAIN);
        
        ConnectedDevices.push_back({"Focuser", QString::fromUtf8(dpFocuser->getDeviceName())});

        systemdevicelist.system_devices[22].DeviceIndiName = QString::fromUtf8(dpFocuser->getDeviceName());
        systemdevicelist.system_devices[22].isBind = true;
        indi_Client->GetAllPropertyName(dpFocuser);
        // indi_Client->syncFocuserPosition(dpFocuser, 0);
        getFocuserParameters();

        int min, max, step, value;
        indi_Client->getFocuserRange(dpFocuser, min, max, step, value);
        if (focuserMaxPosition == -1 && focuserMinPosition == -1)
        {
            focuserMaxPosition = max;
            focuserMinPosition = min;
            Tools::saveParameter("Focuser", "focuserMaxPosition", QString::number(focuserMaxPosition));
            Tools::saveParameter("Focuser", "focuserMinPosition", QString::number(focuserMinPosition));
        }
        emit wsThread->sendMessageToClient("FocuserLimit:" + QString::number(focuserMinPosition) + ":" + QString::number(focuserMaxPosition));
        QString SDKVERSION = "null";
        indi_Client->getFocuserSDKVersion(dpFocuser, SDKVERSION);
        emit wsThread->sendMessageToClient("getSDKVersion:Focuser:" + SDKVERSION);
        Logger::Log("Focuser SDK version: " + SDKVERSION.toStdString(), LogLevel::INFO, DeviceType::MAIN);

        QString DevicePort = "null";
        indi_Client->getDevicePort(dpFocuser, DevicePort);
        emit wsThread->sendMessageToClient("getDevicePort:Focuser:" + DevicePort);
        Logger::Log("Device port for Focuser: " + DevicePort.toStdString(), LogLevel::INFO, DeviceType::MAIN);

        CurrentPosition = FocuserControl_getPosition();
        Logger::Log("Focuser Current Position: " + std::to_string(CurrentPosition), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("FocusPosition:" + QString::number(CurrentPosition) + ":" + QString::number(CurrentPosition));
        Logger::Log("Focuser connected successfully.", LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConnectSuccess:Focuser:" + QString::fromUtf8(dpFocuser->getDeviceName()) + ":" + QString::fromUtf8(dpFocuser->getDriverExec()));
    }

    if (dpCFW == dp)
    {
        Logger::Log("CFW connected after Device(" + QString::fromUtf8(dpCFW->getDeviceName()).toStdString() + ") Connect: " + dpCFW->getDeviceName(), LogLevel::INFO, DeviceType::MAIN);
        
        ConnectedDevices.push_back({"CFW", QString::fromUtf8(dpCFW->getDeviceName())});

        systemdevicelist.system_devices[21].DeviceIndiName = QString::fromUtf8(dpCFW->getDeviceName());
        systemdevicelist.system_devices[21].isBind = true;
        indi_Client->GetAllPropertyName(dpCFW);
        int min, max, pos;
        indi_Client->getCFWPosition(dpCFW, pos, min, max);
        Logger::Log("CFW Position - Min: " + std::to_string(min) + ", Max: " + std::to_string(max) + ", Current: " + std::to_string(pos), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("CFWPositionMax:" + QString::number(max));
        if (Tools::readCFWList(QString::fromUtf8(dpCFW->getDeviceName())) != QString())
        {
            emit wsThread->sendMessageToClient("getCFWList:" + Tools::readCFWList(QString::fromUtf8(dpCFW->getDeviceName())));
        }
        Logger::Log("CFW connected successfully.", LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConnectSuccess:CFW:" + QString::fromUtf8(dpCFW->getDeviceName()) + ":" + QString::fromUtf8(dpCFW->getDriverExec()));
    }

    if (dpGuider == dp)
    {
        Logger::Log("Guider connected after Device(" + QString::fromUtf8(dpGuider->getDeviceName()).toStdString() + ") Connect: " + dpGuider->getDeviceName(), LogLevel::INFO, DeviceType::MAIN);
        
        ConnectedDevices.push_back({"Guider", QString::fromUtf8(dpGuider->getDeviceName())});
        Logger::Log("Guider connected successfully.", LogLevel::INFO, DeviceType::MAIN);
        systemdevicelist.system_devices[1].DeviceIndiName = QString::fromUtf8(dpGuider->getDeviceName());
        systemdevicelist.system_devices[1].isBind = true;
        QString SDKVERSION;
        indi_Client->getCCDSDKVersion(dpGuider, SDKVERSION);
        emit wsThread->sendMessageToClient("getSDKVersion:Guider:" + SDKVERSION);
        Logger::Log("Guider SDK version: " + SDKVERSION.toStdString(), LogLevel::INFO, DeviceType::MAIN);

        emit wsThread->sendMessageToClient("ConnectSuccess:Guider:" + QString::fromUtf8(dpGuider->getDeviceName()) + ":" + QString::fromUtf8(dpGuider->getDriverExec()));
    }

    Tools::saveSystemDeviceList(systemdevicelist);
    // qDebug() << "*** ***  当前系统列表 *** ***";
    // for (int i = 0; i < systemdevicelist.system_devices.size(); i++)
    // {
    //     if (systemdevicelist.system_devices[i].Description != "")
    //     {
    //         qDebug() << "设备类型：" << systemdevicelist.system_devices[i].Description;
    //         qDebug() << "设备名称：" << systemdevicelist.system_devices[i].DeviceIndiName;
    //         qDebug() << "是否绑定：" << systemdevicelist.system_devices[i].isBind;
    //         qDebug() << "驱动名称：" << systemdevicelist.system_devices[i].DriverIndiName;
    //         qDebug() << " *** *** *** *** *** *** ";
    //     }
    // }
    // qDebug() << "*** ***  当前设备列表 *** ***";
    // for (int i = 0; i < indi_Client->GetDeviceCount(); i++)
    // {
    //     qDebug() << "设备名称：" << QString::fromStdString(indi_Client->GetDeviceNameFromList(i));
    //     qDebug() << "驱动名称：" << QString::fromStdString(indi_Client->GetDeviceFromList(i)->getDriverExec());
    //     qDebug() << "是否连接：" << QString::fromStdString(std::to_string(indi_Client->GetDeviceFromList(i)->isConnected()));
    //     qDebug() << " *** *** *** *** *** *** ";
    // }
}


bool MainWindow::hasProp(INDI::BaseDevice *dev, const char *prop)
{
    return dev && dev->getProperty(prop) != nullptr;
}

// 工具函数：检查多个属性是否存在其中之一
bool MainWindow::hasAnyProp(INDI::BaseDevice *dev, std::initializer_list<const char*> props)
{
    for (auto p : props)
    {
        if (hasProp(dev, p))
            return true;
    }
    return false;
}

bool MainWindow::isDSLR(INDI::BaseDevice *device)
{
    if (!device) return false;

    // 转小写便于匹配
    auto toLower = [](QString s){ return s.toLower(); };
    QString drvExec = toLower(QString::fromUtf8(device->getDriverExec()));
    QString devName = toLower(QString::fromUtf8(device->getDeviceName()));

    auto nameHas = [&](const QString& key){
        return drvExec.contains(key) || devName.contains(key);
    };

    bool nameLooksDSLR = nameHas("gphoto") || nameHas("dslr");

    // 1) 反证：典型 SDK/制冷相机属性，若存在则优先判定为非 DSLR
    bool looksLikeSDKCam = hasAnyProp(device, {
        "CCD_COOLER", "CCD_COOLER_MODE", "CCD_COOLER_POWER", "CCD_HUMIDITY",
        "CCD_GAIN", "CCD_OFFSET", "USB_TRAFFIC", "USB_BUFFER",
        "SDK_VERSION", "READ_MODE"
    });
    if (looksLikeSDKCam)
    {
        Logger::Log("SDK/Coooler/Gain type properties found, treat as non-DSLR.",
                    LogLevel::INFO, DeviceType::MAIN);
        return false;
    }

    // 2) 正证：单反/微单常见属性
    bool dslrProps = hasAnyProp(device, {
        "ISO", "CCD_ISO", "APERTURE", "WB", "WHITE_BALANCE",
        "CAPTURE_TARGET", "IMAGE_FORMAT", "LIVEVIEW", "LIVE_VIEW", "FOCUS_MODE"
    });

    if (dslrProps)
    {
        Logger::Log("Found DSLR-specific properties, treat as DSLR.",
                    LogLevel::INFO, DeviceType::MAIN);
        return true;
    }

    // 3) 兜底：驱动名提示
    if (nameLooksDSLR)
    {
        Logger::Log("Driver name contains DSLR/GPhoto, treat as DSLR.",
                    LogLevel::INFO, DeviceType::MAIN);
        return true;
    }

    return false;
}

void MainWindow::disconnectIndiServer(MyClient *client)
{
    Logger::Log("disconnectIndiServer start ...", LogLevel::INFO, DeviceType::MAIN);
    // 防御性检查：客户端指针为空则直接返回，避免段错误
    if (client == nullptr)
    {
        Logger::Log("disconnectIndiServer | client is nullptr", LogLevel::ERROR, DeviceType::MAIN);
        Tools::stopIndiDriverAll(drivers_list);
        ConnectDriverList.clear();
        return;
    }

    int deviceCount = client->GetDeviceCount();
    if (deviceCount > 0)
    {
        for (int i = 0; i < deviceCount; i++)
        {
            INDI::BaseDevice *device = client->GetDeviceFromList(i);
            if (device == nullptr)
            {
                Logger::Log("disconnectAllDevice | Device at index " + std::to_string(i) + " is nullptr", LogLevel::WARNING, DeviceType::MAIN);
                continue;
            }

            if (device->isConnected())
            {
                const char *devName = device->getDeviceName();
                QString qName = devName ? QString::fromUtf8(devName) : QString("UnknownDevice");

                client->disconnectDevice(devName ? devName : "");
                int num = 0;
                while (device->isConnected())
                {
                    Logger::Log("disconnectAllDevice | Waiting for disconnect device (" + qName.toStdString() + ") finish...", LogLevel::INFO, DeviceType::MAIN);
                    sleep(1);
                    num++;

                    if (num > 10)
                    {
                        Logger::Log("disconnectAllDevice | device (" + qName.toStdString() + ") disconnect failed.", LogLevel::WARNING, DeviceType::MAIN);
                        break;
                    }
                }
                Logger::Log("disconnectAllDevice | device (" + qName.toStdString() + ") disconnected successfully.", LogLevel::INFO, DeviceType::MAIN);
            }
        }
    }
    else
    {
        Logger::Log("disconnectIndiServer | no devices to disconnect (device count = 0)", LogLevel::INFO, DeviceType::MAIN);
    }

    Tools::stopIndiDriverAll(drivers_list);
    ConnectDriverList.clear();

    client->ClearDevices();
    client->disconnectServer();
    int k = 10;
    while (k--)
    {
        if (!client->isServerConnected())
        {
            Logger::Log("Server disconnected successfully.", LogLevel::INFO, DeviceType::MAIN);
            break;
        }
        sleep(1);
        // qApp->processEvents();
        Logger::Log("Waiting for server to disconnect...", LogLevel::INFO, DeviceType::MAIN);
    }
    Logger::Log("disconnectServer finished.", LogLevel::INFO, DeviceType::MAIN);
    if (indi_Client != nullptr)
    {
        indi_Client->PrintDevices();
    }
}

bool MainWindow::connectIndiServer(MyClient *client)
{
    Logger::Log("connectIndiServer start ...", LogLevel::INFO, DeviceType::MAIN);
    client->setConnectionTimeout(3, 0);
    Logger::Log("connectIndiServer | clear device list ...", LogLevel::INFO, DeviceType::MAIN);
    client->ClearDevices(); // clear device list
    Logger::Log("connectIndiServer | connect server ...", LogLevel::INFO, DeviceType::MAIN);
    client->connectServer();
    int k = 10;
    while (k--)
    {
        if (client->isServerConnected() == true)
        {
            break;
        }
        sleep(1);
        // qApp->processEvents();
        Logger::Log("connectIndiServer | waiting for client connected ...", LogLevel::INFO, DeviceType::MAIN);
    }
    sleep(1);
    client->PrintDevices();
    Logger::Log("connectIndiServer finished.", LogLevel::INFO, DeviceType::MAIN);
    return true;
}

void MainWindow::ClearSystemDeviceList()
{
    Logger::Log("ClearSystemDeviceList start ...", LogLevel::INFO, DeviceType::MAIN);
    for (int i = 0; i < systemdevicelist.system_devices.size(); i++)
    {
        systemdevicelist.system_devices[i].DeviceIndiGroup = -1;
        systemdevicelist.system_devices[i].DeviceIndiName = "";
        systemdevicelist.system_devices[i].DriverFrom = "";
        // systemdevicelist.system_devices[i].DriverIndiName = "";
        systemdevicelist.system_devices[i].isConnect = false;
        systemdevicelist.system_devices[i].dp = NULL;
        systemdevicelist.system_devices[i].isBind = false;
        // systemdevicelist.system_devices[i].Description = "";
    }
    Logger::Log("ClearSystemDeviceList finished.", LogLevel::INFO, DeviceType::MAIN);
    Tools::printSystemDeviceList(systemdevicelist);
}

void MainWindow::INDI_Capture(int Exp_times)
{
    Logger::Log("INDI_Capture start ...", LogLevel::INFO, DeviceType::CAMERA);
    glIsFocusingLooping = false;
    isSavePngSuccess = false;
    double expTime_sec;
    expTime_sec = (double)Exp_times / 1000;
    Logger::Log("INDI_Capture | convert Exp_times to seconds:" + std::to_string(expTime_sec), LogLevel::INFO, DeviceType::CAMERA);

    if (dpMainCamera)
    {
        glMainCameraStatu = "Exposuring";
        Logger::Log("INDI_Capture | check Main Camera Status(glMainCameraStatu):" + glMainCameraStatu.toStdString(), LogLevel::INFO, DeviceType::CAMERA);

        int value, min, max;
        uint32_t ret = indi_Client->getCCDGain(dpMainCamera, value, min, max);
        if (ret != QHYCCD_SUCCESS)
        {
            Logger::Log("INDI_Capture | indi getCCDGain | failed", LogLevel::WARNING, DeviceType::CAMERA);
        }
        Logger::Log("INDI_Capture | indi getCCDGain | value:" + std::to_string(value) + ", min:" + std::to_string(min) + ", max:" + std::to_string(max), LogLevel::INFO, DeviceType::CAMERA);
        int BINX, BINY, BINXMAX, BINYMAX;
        ret = indi_Client->getCCDBinning(dpMainCamera, BINX, BINY, BINXMAX, BINYMAX);
        if (ret != QHYCCD_SUCCESS)
        {
            Logger::Log("INDI_Capture | indi getCCDBinning | failed", LogLevel::WARNING, DeviceType::CAMERA);
        }
        Logger::Log("INDI_Capture | indi getCCDBinning | BINX:" + std::to_string(BINX) + ", BINY:" + std::to_string(BINY) + ", BINXMAX:" + std::to_string(BINXMAX) + ", BINYMAX:" + std::to_string(BINYMAX), LogLevel::INFO, DeviceType::CAMERA);
        ret = indi_Client->getCCDOffset(dpMainCamera, value, min, max);
        if (ret != QHYCCD_SUCCESS)
        {
            Logger::Log("INDI_Capture | indi getCCDOffset | failed", LogLevel::WARNING, DeviceType::CAMERA);
        }
        Logger::Log("INDI_Capture | indi getCCDOffset | value:" + std::to_string(value) + ", min:" + std::to_string(min) + ", max:" + std::to_string(max), LogLevel::INFO, DeviceType::CAMERA);
        ret = indi_Client->resetCCDFrameInfo(dpMainCamera);
        if (ret != QHYCCD_SUCCESS)
        {
            Logger::Log("INDI_Capture | indi resetCCDFrameInfo | failed", LogLevel::WARNING, DeviceType::CAMERA);
        }
        Logger::Log("INDI_Capture | indi resetCCDFrameInfo", LogLevel::INFO, DeviceType::CAMERA);
        emit wsThread->sendMessageToClient("MainCameraSize:" + QString::number(glMainCCDSizeX) + ":" + QString::number(glMainCCDSizeY));
        Logger::Log("INDI_Capture | sendMessageToClient | MainCameraSize:" + QString::number(glMainCCDSizeX).toStdString() + ":" + QString::number(glMainCCDSizeY).toStdString(), LogLevel::INFO, DeviceType::CAMERA);
        ret = indi_Client->takeExposure(dpMainCamera, expTime_sec);
        if (ret != QHYCCD_SUCCESS)
        {
            Logger::Log("INDI_Capture | indi takeExposure | failed", LogLevel::WARNING, DeviceType::CAMERA);
        }
        Logger::Log("INDI_Capture | indi start takeExposure, expTime_sec:" + std::to_string(expTime_sec), LogLevel::INFO, DeviceType::CAMERA);
    }
    else
    {
        Logger::Log("INDI_Capture | dpMainCamera is NULL", LogLevel::WARNING, DeviceType::CAMERA);
        ShootStatus = "IDLE";
    }
    Logger::Log("INDI_Capture finished.", LogLevel::INFO, DeviceType::CAMERA);
}

void MainWindow::INDI_AbortCapture()
{
    Logger::Log("INDI_AbortCapture start ...", LogLevel::INFO, DeviceType::CAMERA);
    glMainCameraStatu = "IDLE";
    Logger::Log("INDI_AbortCapture | glMainCameraStatu:" + glMainCameraStatu.toStdString(), LogLevel::INFO, DeviceType::CAMERA);
    if (dpMainCamera)
    {
        indi_Client->setCCDAbortExposure(dpMainCamera);
        ShootStatus = "IDLE";
        Logger::Log("INDI_AbortCapture | ShootStatus:" + ShootStatus.toStdString(), LogLevel::INFO, DeviceType::CAMERA);
    }
    Logger::Log("INDI_AbortCapture finished.", LogLevel::INFO, DeviceType::CAMERA);
}

void MainWindow::FocusingLooping()
{
    Logger::Log("FocusingLooping start ...", LogLevel::DEBUG, DeviceType::FOCUSER);
    // 检查相机是否连接，如果未连接则记录警告并返回
    if (dpMainCamera == NULL)
    {
        Logger::Log("FocusingLooping | dpMainCamera is NULL", LogLevel::WARNING, DeviceType::FOCUSER);
        return;
    }

    isSavePngSuccess = false;

    glIsFocusingLooping = true;
    Logger::Log("FocusingLooping | glIsFocusingLooping:" + std::to_string(glIsFocusingLooping), LogLevel::DEBUG, DeviceType::FOCUSER);
    // 如果相机状态为"显示中"，则开始处理曝光
    if (glMainCameraStatu != "Exposuring")
    {
        double expTime_sec;
        expTime_sec = (double)glExpTime / 1000; // 将曝光时间从毫秒转换为秒

        glMainCameraStatu = "Exposuring";
        Logger::Log("FocusingLooping | glMainCameraStatu:" + glMainCameraStatu.toStdString(), LogLevel::DEBUG, DeviceType::FOCUSER);

        QSize cameraResolution{glMainCCDSizeX, glMainCCDSizeY};
        QSize ROI{BoxSideLength, BoxSideLength};

        Logger::Log("FocusingLooping |当前ROI值 ROI_x:" + std::to_string(roiAndFocuserInfo["ROI_x"]) + ", ROI_y:" + std::to_string(roiAndFocuserInfo["ROI_y"]), LogLevel::DEBUG, DeviceType::FOCUSER);
        int cameraX = static_cast<int>(roiAndFocuserInfo["ROI_x"]);
        int cameraY = static_cast<int>(roiAndFocuserInfo["ROI_y"]);

        // 确保 cameraX 和 cameraY 是偶数
        if (cameraX % 2 != 0)
        {
            cameraX += 1;
        }
        if (cameraY % 2 != 0)
        {
            cameraY += 1;
        }

        // 使用缩放坐标（乘以 binning）并在缩放空间内裁剪，允许等于边界
        int scaledX = cameraX * glMainCameraBinning;
        int scaledY = cameraY * glMainCameraBinning;
        if (scaledX < 0) scaledX = 0;
        if (scaledY < 0) scaledY = 0;
        if (BoxSideLength > glMainCCDSizeX) BoxSideLength = glMainCCDSizeX;
        if (BoxSideLength > glMainCCDSizeY) BoxSideLength = glMainCCDSizeY;
        ROI = QSize(BoxSideLength, BoxSideLength);
        if (scaledX > glMainCCDSizeX - ROI.width()) scaledX = glMainCCDSizeX - ROI.width();
        if (scaledY > glMainCCDSizeY - ROI.height()) scaledY = glMainCCDSizeY - ROI.height();

        if (scaledX <= glMainCCDSizeX - ROI.width() && scaledY <= glMainCCDSizeY - ROI.height())
        {
            Logger::Log("FocusingLooping | set Camera ROI x:" + std::to_string(cameraX) + ", y:" + std::to_string(cameraY) + ", width:" + std::to_string(BoxSideLength) + ", height:" + std::to_string(BoxSideLength), LogLevel::DEBUG, DeviceType::FOCUSER);
            // 将裁剪后的缩放坐标反馈为未缩放 ROI，保持区域大小不变，仅位置贴边
            if (glMainCameraBinning > 0) {
                roiAndFocuserInfo["ROI_x"] = static_cast<double>(scaledX) / glMainCameraBinning;
                roiAndFocuserInfo["ROI_y"] = static_cast<double>(scaledY) / glMainCameraBinning;
            }
            indi_Client->setCCDFrameInfo(dpMainCamera, scaledX, scaledY, BoxSideLength, BoxSideLength); // 设置相机的曝光区域
            indi_Client->takeExposure(dpMainCamera, expTime_sec);                                       // 进行曝光
            Logger::Log("FocusingLooping | takeExposure, expTime_sec:" + std::to_string(expTime_sec), LogLevel::DEBUG, DeviceType::FOCUSER);
        }
        else
        {
            Logger::Log("FocusingLooping | Too close to the edge, please reselect the area.", LogLevel::WARNING, DeviceType::FOCUSER); // 如果区域太靠近边缘，记录警告并调整
            if (scaledX + ROI.width() > glMainCCDSizeX)
                scaledX = glMainCCDSizeX - ROI.width();
            if (scaledY + ROI.height() > glMainCCDSizeY)
                scaledY = glMainCCDSizeY - ROI.height();

            // 将修正后的缩放坐标反馈为未缩放 ROI，区域大小不变，仅位置贴边
            if (glMainCameraBinning > 0) {
                roiAndFocuserInfo["ROI_x"] = static_cast<double>(scaledX) / glMainCameraBinning;
                roiAndFocuserInfo["ROI_y"] = static_cast<double>(scaledY) / glMainCameraBinning;
            }
 
            indi_Client->setCCDFrameInfo(dpMainCamera, scaledX, scaledY, ROI.width(), ROI.height()); // 重新设置曝光区域并进行曝光
            indi_Client->takeExposure(dpMainCamera, expTime_sec);
        }
    }
    else
    {
        emit wsThread->sendMessageToClient("startFocusLoopFailed:Wait Take Picture Finish!");
    }
    Logger::Log("FocusingLooping finished.", LogLevel::DEBUG, DeviceType::FOCUSER);
}



void MainWindow::InitPHD2()
{
    Logger::Log("InitPHD2 start ...", LogLevel::INFO, DeviceType::MAIN);
    isGuideCapture = true;

    if (!cmdPHD2) cmdPHD2 = new QProcess();
    static bool phdSignalsConnected = false;
    if (!phdSignalsConnected)
    {
        connect(cmdPHD2, QOverload<int, QProcess::ExitStatus>::of(&QProcess::finished),
                this, &MainWindow::onPhd2Exited);
        connect(cmdPHD2, &QProcess::errorOccurred,
                this, &MainWindow::onPhd2Error);
        phdSignalsConnected = true;
    }

    bool connected = false;
    int retryCount = 3; // 设定重试次数
    while (retryCount > 0 && !connected)
    {
        // 启动前强制结束残留进程并清理共享内存
        // 以避免“初次启动时无法关闭导致启动失败”的问题
        // 1) 若之前有 QProcess 实例，先尝试优雅结束并回收，避免僵尸进程
        if (cmdPHD2->state() != QProcess::NotRunning) {
            cmdPHD2->terminate();
            if (!cmdPHD2->waitForFinished(1500)) {
                cmdPHD2->kill();
                cmdPHD2->waitForFinished(1000);
            }
        }
        // 2) 系统级强杀（多种匹配方式）
        // 注意：有的系统进程名是 phd2.bin，这里同时匹配 phd2 和 phd2.bin
        QProcess::execute("pkill", QStringList() << "-TERM" << "-x" << "phd2");
        QProcess::execute("pkill", QStringList() << "-TERM" << "-x" << "phd2.bin");
        QThread::msleep(200);
        QProcess::execute("pkill", QStringList() << "-KILL" << "-x" << "phd2");
        QProcess::execute("pkill", QStringList() << "-KILL" << "-x" << "phd2.bin");
        QThread::msleep(100);
        // 3) 宽匹配（包含路径/命令行）
        QProcess::execute("pkill", QStringList() << "-TERM" << "-f" << "phd2");
        QThread::msleep(150);
        QProcess::execute("pkill", QStringList() << "-KILL" << "-f" << "phd2");
        QThread::msleep(150);
        // 4) 轮询确认已无残留进程（最多 1s）
        {
            QElapsedTimer waitKill;
            waitKill.start();
            while (waitKill.elapsed() < 1000) {
                int rc = QProcess::execute("pgrep", QStringList() << "-f" << "phd2");
                if (rc != 0) break; // 无匹配
                QThread::msleep(100);
            }
        }

        // 5) 启动前清空 PHD2 日志目录，以规避“损坏/异常 GuidingLog 导致启动卡死”的问题
        //    目标目录：/home/quarcs/Documents/PHD2
        {
            const QString phd2LogDirPath = QStringLiteral("~/Documents/PHD2");
            QDir phd2LogDir(phd2LogDirPath);
            if (phd2LogDir.exists())
            {
                // 递归删除整个日志目录及其内容，然后重新创建一个空目录
                if (!phd2LogDir.removeRecursively())
                {
                    Logger::Log("InitPHD2 | failed to clear PHD2 log dir: " + phd2LogDirPath.toStdString(),
                                LogLevel::WARNING, DeviceType::MAIN);
                }
            }
            // 确保目录最终存在（即使之前不存在或被删除）
            if (!phd2LogDir.mkpath("."))
            {
                Logger::Log("InitPHD2 | failed to recreate PHD2 log dir: " + phd2LogDirPath.toStdString(),
                            LogLevel::WARNING, DeviceType::MAIN);
            }
            else
            {
                Logger::Log("InitPHD2 | PHD2 log dir cleared: " + phd2LogDirPath.toStdString(),
                            LogLevel::INFO, DeviceType::MAIN);
            }
        }
        // 清理共享内存段（key=0x90）
        key_t cleanup_key = 0x90;
        int cleanup_id = shmget(cleanup_key, BUFSZ_PHD, 0666);
        if (cleanup_id != -1) shmctl(cleanup_id, IPC_RMID, NULL);

        // 重新生成共享内存，避免之前的进程遗留问题
        key_phd = 0x90;                                           // 重新设置共享内存的键值
        shmid_phd = shmget(key_phd, BUFSZ_PHD, IPC_CREAT | 0666); // 获取共享内存
        if (shmid_phd < 0)
        {
            Logger::Log("InitPHD2 | shared memory phd shmget ERROR", LogLevel::ERROR, DeviceType::MAIN);
            continue;
        }

        // 映射共享内存
        sharedmemory_phd = (char *)shmat(shmid_phd, NULL, 0);
        if (sharedmemory_phd == NULL)
        {
            Logger::Log("InitPHD2 | shared memory phd map ERROR", LogLevel::ERROR, DeviceType::MAIN);
            continue;
        }

        // 读取共享内存数据（避免将共享内存当作以\\0 结尾字符串打印，可能越界）
        Logger::Log("InitPHD2 | shared memory mapped", LogLevel::INFO, DeviceType::MAIN);

        // 启动 phd2 进程（显式指定实例号 1）
        cmdPHD2->start("phd2", QStringList() << "-i" << "1");
        phd2ExpectedRunning = true;

        // 等待最多 10 秒尝试连接
        QElapsedTimer t;
        t.start();
        while (t.elapsed() < 10000)
        {
            usleep(10000);
            qApp->processEvents();
            if (connectPHD() == true)
            {
                connected = true;
                break;
            }
        }

        if (!connected)
        {
            Logger::Log("InitPHD2 | Failed to connect to phd2. Retrying...", LogLevel::WARNING, DeviceType::MAIN);
            retryCount--; // 如果连接失败，重试次数减 1
        }
    }

    if (!connected)
    {
        Logger::Log("InitPHD2 | Failed to connect to phd2 after retries.", LogLevel::ERROR, DeviceType::MAIN);
    }

    Logger::Log("InitPHD2 finished.", LogLevel::INFO, DeviceType::MAIN);
}

void MainWindow::onPhd2Exited(int exitCode, QProcess::ExitStatus exitStatus)
{
    Logger::Log("PHD2 exited. code=" + std::to_string(exitCode) +
                " status=" + std::to_string((int)exitStatus), LogLevel::WARNING, DeviceType::GUIDER);
    if (phd2ExpectedRunning)
    {
        // 进程异常结束时，尝试发送一次“停止循环拍摄”命令以收敛前端状态
        call_phd_StopLooping();
        
        // 通知前端状态：循环曝光已关闭
        emit wsThread->sendMessageToClient("GuiderLoopExpStatus:false");
        phd2ExpectedRunning = false;
        // 清理共享内存段，避免前端继续读到旧数据
        key_t key = 0x90;
        int id = shmget(key, BUFSZ_PHD, 0666);
        if (id != -1) shmctl(id, IPC_RMID, NULL);
        // 提示前端是否重启
        emit wsThread->sendMessageToClient("PHD2ClosedUnexpectedly:是否重新启动PHD2?");
    }
}

void MainWindow::onPhd2Error(QProcess::ProcessError error)
{
    Logger::Log("PHD2 process error: " + std::to_string((int)error), LogLevel::ERROR, DeviceType::GUIDER);
    if (phd2ExpectedRunning)
    {
        // 进程错误时，尝试发送“停止循环拍摄”命令以确保状态一致
        call_phd_StopLooping();
        // 通知前端状态：循环曝光已关闭
        emit wsThread->sendMessageToClient("GuiderLoopExpStatus:false");
        phd2ExpectedRunning = false;
        key_t key = 0x90;
        int id = shmget(key, BUFSZ_PHD, 0666);
        if (id != -1) shmctl(id, IPC_RMID, NULL);
        emit wsThread->sendMessageToClient("PHD2ClosedUnexpectedly:是否重新启动PHD2?");
    }
}

void MainWindow::disconnectFocuserIfConnected()
{
    if (dpFocuser && dpFocuser->isConnected())
    {
        DisconnectDevice(indi_Client, dpFocuser->getDeviceName(), "Focuser");
    }
}

bool MainWindow::connectPHD(void)
{
    Logger::Log("connectPHD start ...", LogLevel::INFO, DeviceType::MAIN);
    QString versionName = "";
    call_phd_GetVersion(versionName);

    Logger::Log("connectPHD | version:" + versionName.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    if (versionName != "")
    {
        // init stellarium operation
        Logger::Log("connectPHD Success!", LogLevel::INFO, DeviceType::MAIN);
        return true;
    }
    else
    {
        Logger::Log("connectPHD | there is no openPHD2 running", LogLevel::ERROR, DeviceType::MAIN);
        Logger::Log("connectPHD failed.", LogLevel::ERROR, DeviceType::MAIN);
        return false;
    }
}

bool MainWindow::call_phd_GetVersion(QString &versionName)
{
    Logger::Log("call_phd_GetVersion start ...", LogLevel::INFO, DeviceType::MAIN);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_GetVersion | shared memory not ready", LogLevel::ERROR, DeviceType::MAIN);
        versionName = "";
        return false;
    }
    
    unsigned int baseAddress;
    unsigned int vendcommand;
    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x01;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    QElapsedTimer t;
    t.start();

    // 放宽首次连接等待时长，避免 PHD2 在树莓派等设备上启动/初始化较慢导致的超时
    // 最长等待 10 秒，让 PHD2 有充分时间写回版本信息
    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 10000)
    {
        QThread::msleep(2);
    }

    // 如果超过 10 秒仍未收到响应，则认为超时
    if (t.elapsed() >= 10000)
    {
        versionName = "";
        Logger::Log("call_phd_GetVersion | timeout", LogLevel::ERROR, DeviceType::MAIN);
        Logger::Log("call_phd_GetVersion failed.", LogLevel::ERROR, DeviceType::MAIN);
        return false;
    }
    else
    {
        unsigned char addr = 0;
        uint16_t length;
        memcpy(&length, sharedmemory_phd + baseAddress + addr, sizeof(uint16_t));
        addr = addr + sizeof(uint16_t);
        // qDebug()<<length;

        if (length > 0 && length < 1024)
        {
            for (int i = 0; i < length; i++)
            {
                versionName.append(sharedmemory_phd[baseAddress + addr + i]);
            }
            Logger::Log("call_phd_GetVersion | version:" + versionName.toStdString(), LogLevel::INFO, DeviceType::MAIN);
            Logger::Log("call_phd_GetVersion success.", LogLevel::INFO, DeviceType::MAIN);
            return true;
            // qDebug()<<versionName;
        }
        else
        {
            versionName = "";
            Logger::Log("call_phd_GetVersion | version is empty", LogLevel::ERROR, DeviceType::MAIN);
            Logger::Log("call_phd_GetVersion failed.", LogLevel::ERROR, DeviceType::MAIN);
            return false;
        }
    }
}

uint32_t MainWindow::call_phd_StartLooping(void)
{
    Logger::Log("call_phd_StartLooping start ...", LogLevel::INFO, DeviceType::GUIDER);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_StartLooping | shared memory not ready", LogLevel::ERROR, DeviceType::GUIDER);
        return false;
    }
    
    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x03;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // 避免忙等占满 CPU，增加适度休眠
        QThread::msleep(100);
    }
    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_StartLooping | timeout", LogLevel::ERROR, DeviceType::GUIDER);
        Logger::Log("call_phd_StartLooping failed.", LogLevel::ERROR, DeviceType::GUIDER);
        return false; // timeout
    }
    else
    {
        Logger::Log("call_phd_StartLooping success.", LogLevel::INFO, DeviceType::GUIDER);
        return true;
    }
}

uint32_t MainWindow::call_phd_StopLooping(void)
{
    Logger::Log("call_phd_StopLooping start ...", LogLevel::INFO, DeviceType::GUIDER);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_StopLooping | shared memory not ready", LogLevel::ERROR, DeviceType::GUIDER);
        return false;
    }
    
    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x04;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // 避免忙等占满 CPU，增加适度休眠
        QThread::msleep(100);
    }
    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_StopLooping | timeout", LogLevel::ERROR, DeviceType::GUIDER);
        Logger::Log("call_phd_StopLooping failed.", LogLevel::ERROR, DeviceType::GUIDER);
        return false; // timeout
    }
    else
    {
        Logger::Log("call_phd_StopLooping success.", LogLevel::INFO, DeviceType::GUIDER);
        return true;
    }
}

uint32_t MainWindow::call_phd_AutoFindStar(void)
{
    Logger::Log("call_phd_AutoFindStar start ...", LogLevel::INFO, DeviceType::GUIDER);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_AutoFindStar | shared memory not ready", LogLevel::ERROR, DeviceType::GUIDER);
        return false;
    }
    
    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x05;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    }
    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_AutoFindStar | timeout", LogLevel::ERROR, DeviceType::GUIDER);
        Logger::Log("call_phd_AutoFindStar failed.", LogLevel::ERROR, DeviceType::GUIDER);
        return false; // timeout
    }
    else
    {
        Logger::Log("call_phd_AutoFindStar success.", LogLevel::INFO, DeviceType::GUIDER);
        return true;
    }
}

uint32_t MainWindow::call_phd_StartGuiding(void)
{
    Logger::Log("call_phd_StartGuiding start ...", LogLevel::INFO, DeviceType::GUIDER);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_StartGuiding | shared memory not ready", LogLevel::ERROR, DeviceType::GUIDER);
        return false;
    }
    
    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x06;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    }
    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_StartGuiding | timeout", LogLevel::ERROR, DeviceType::GUIDER);
        Logger::Log("call_phd_StartGuiding failed.", LogLevel::ERROR, DeviceType::GUIDER);
        // 在启动导星失败时，发送关闭循环拍摄的命令
        // call_phd_StopLooping();
        // 通知前端状态：循环曝光已关闭
        // emit wsThread->sendMessageToClient("GuiderLoopExpStatus:false");
        isGuiding = false;
        return false; // timeout
    }
    else
    {
        Logger::Log("call_phd_StartGuiding success.", LogLevel::INFO, DeviceType::GUIDER);
        return true;
    }
}

uint32_t MainWindow::call_phd_StopGuiding(void)
{
    Logger::Log("call_phd_StopGuiding start ...", LogLevel::INFO, DeviceType::GUIDER);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_StopGuiding | shared memory not ready", LogLevel::ERROR, DeviceType::GUIDER);
        return false;
    }

    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x17; // 与 PHD2 端 myframe.cpp 中定义的“Stop Guiding Only”命令码保持一致

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    }
    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_StopGuiding | timeout", LogLevel::ERROR, DeviceType::GUIDER);
        Logger::Log("call_phd_StopGuiding failed.", LogLevel::ERROR, DeviceType::GUIDER);
        return false; // timeout
    }
    else
    {
        Logger::Log("call_phd_StopGuiding success.", LogLevel::INFO, DeviceType::GUIDER);
        return true;
    }
}

void MainWindow::pauseGuidingBeforeMountMove()
{
    // 仅在当前逻辑导星开关为 ON 时，才在移动前主动停止导星，并记录状态以便之后恢复
    wasGuidingBeforeMountMove = false;

    if (isGuiding)
    {
        Logger::Log("pauseGuidingBeforeMountMove | guiding is ON, stop guiding before mount move.",
                    LogLevel::INFO, DeviceType::GUIDER);
        wasGuidingBeforeMountMove = true;
        // 仅停止导星，不关闭循环曝光，避免影响 PHD2 的 Loop 按钮语义
        call_phd_StopGuiding();
        // 这里不去强制修改 isGuiding / 循环开关，由 PHD2 端自身状态机与前端 UI 控制
    }
}

void MainWindow::resumeGuidingAfterMountMove()
{
    if (!wasGuidingBeforeMountMove)
    {
        // 移动前没有在导星，无需恢复
        return;
    }

    Logger::Log("resumeGuidingAfterMountMove | mount move finished, resume guiding.",
                LogLevel::INFO, DeviceType::GUIDER);

    // 参考 GuiderSwitch=true 的逻辑：如需清校准则先清，再根据是否已选星决定是否自动寻星，最后启动导星
    if (ClearCalibrationData)
    {
        ClearCalibrationData = false;
        call_phd_ClearCalibration();
        Logger::Log("resumeGuidingAfterMountMove | clear calibration data before restart guiding.",
                    LogLevel::INFO, DeviceType::GUIDER);
    }

    if (!glPHD_isSelected)
    {
        Logger::Log("resumeGuidingAfterMountMove | no selected star, call AutoFindStar before guiding.",
                    LogLevel::INFO, DeviceType::GUIDER);
        call_phd_AutoFindStar();
    }

    call_phd_StartGuiding();
    emit wsThread->sendMessageToClient("GuiderUpdateStatus:1");
}

uint32_t MainWindow::call_phd_checkStatus(unsigned char &status)
{
    Logger::Log("call_phd_checkStatus start ...", LogLevel::DEBUG, DeviceType::GUIDER);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_checkStatus | shared memory not ready", LogLevel::ERROR, DeviceType::GUIDER);
        status = 0;
        return false;
    }
    
    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x07;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    // wait stellarium finished this task
    QElapsedTimer t;
    t.start();
    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    } // wait stellarium run end

    if (t.elapsed() >= 500)
    {
        // timeout
        status = 0;
        Logger::Log("call_phd_checkStatus | timeout", LogLevel::ERROR, DeviceType::GUIDER);
        Logger::Log("call_phd_checkStatus failed.", LogLevel::ERROR, DeviceType::GUIDER);
        return false;
    }

    else
    {
        status = sharedmemory_phd[3];
        Logger::Log("call_phd_checkStatus | status:" + std::to_string(status), LogLevel::DEBUG, DeviceType::GUIDER);
        Logger::Log("call_phd_checkStatus success.", LogLevel::DEBUG, DeviceType::GUIDER);
        return true;
    }
}

uint32_t MainWindow::call_phd_setExposureTime(unsigned int expTime)
{
    Logger::Log("call_phd_setExposureTime start ...", LogLevel::INFO, DeviceType::GUIDER);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_setExposureTime | shared memory not ready", LogLevel::ERROR, DeviceType::GUIDER);
        return false;
    }
    
    unsigned int vendcommand;
    unsigned int baseAddress;
    Logger::Log("call_phd_setExposureTime | expTime:" + std::to_string(expTime), LogLevel::INFO, DeviceType::GUIDER);

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x0b;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    unsigned char addr = 0;
    memcpy(sharedmemory_phd + baseAddress + addr, &expTime, sizeof(unsigned int));
    addr = addr + sizeof(unsigned int);

    sharedmemory_phd[0] = 0x01; // enable command

    // wait stellarium finished this task
    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    } // wait stellarium run end

    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_setExposureTime | timeout", LogLevel::ERROR, DeviceType::GUIDER);
        Logger::Log("call_phd_setExposureTime failed.", LogLevel::ERROR, DeviceType::GUIDER);
        return QHYCCD_ERROR; // timeout
    }
    else
    {
        Logger::Log("call_phd_setExposureTime success.", LogLevel::INFO, DeviceType::GUIDER);
        return QHYCCD_SUCCESS;
    }
}

uint32_t MainWindow::call_phd_whichCamera(std::string Camera)
{
    Logger::Log("call_phd_whichCamera start ...", LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("call_phd_whichCamera | Camera:" + Camera, LogLevel::INFO, DeviceType::MAIN);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_whichCamera | shared memory not ready", LogLevel::ERROR, DeviceType::MAIN);
        return QHYCCD_ERROR;
    }

    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x0d;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    int length = Camera.length() + 1;

    unsigned char addr = 0;
    // memcpy(sharedmemory_phd + baseAddress + addr, &index, sizeof(int));
    // addr = addr + sizeof(int);
    memcpy(sharedmemory_phd + baseAddress + addr, &length, sizeof(int));
    addr = addr + sizeof(int);
    memcpy(sharedmemory_phd + baseAddress + addr, Camera.c_str(), length);
    addr = addr + length;

    // wait stellarium finished this task
    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    } // wait stellarium run end

    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_whichCamera | timeout", LogLevel::ERROR, DeviceType::MAIN);
        Logger::Log("call_phd_whichCamera failed.", LogLevel::ERROR, DeviceType::MAIN);
        return QHYCCD_ERROR; // timeout
    }
    else
    {
        Logger::Log("call_phd_whichCamera success.", LogLevel::INFO, DeviceType::MAIN);
        return QHYCCD_SUCCESS;
    }
}

uint32_t MainWindow::call_phd_ChackControlStatus(int sdk_num)
{
    Logger::Log("call_phd_ChackControlStatus start ...", LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("call_phd_ChackControlStatus | sdk_num:" + std::to_string(sdk_num), LogLevel::INFO, DeviceType::MAIN);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_ChackControlStatus | shared memory not ready", LogLevel::ERROR, DeviceType::MAIN);
        return false;
    }

    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x0e;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    unsigned char addr = 0;
    memcpy(sharedmemory_phd + baseAddress + addr, &sdk_num, sizeof(int));
    addr = addr + sizeof(int);

    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    }
    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_ChackControlStatus | timeout", LogLevel::ERROR, DeviceType::MAIN);
        Logger::Log("call_phd_ChackControlStatus failed.", LogLevel::ERROR, DeviceType::MAIN);
        return false; // timeout
    }
    else
    {
        Logger::Log("call_phd_ChackControlStatus success.", LogLevel::INFO, DeviceType::MAIN);
        return true;
    }
}

uint32_t MainWindow::call_phd_ClearCalibration(void)
{
    Logger::Log("call_phd_ClearCalibration start ...", LogLevel::INFO, DeviceType::GUIDER);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_ClearCalibration | shared memory not ready", LogLevel::ERROR, DeviceType::GUIDER);
        return false;
    }
    
    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x02;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    }
    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_ClearCalibration | timeout", LogLevel::ERROR, DeviceType::GUIDER);
        Logger::Log("call_phd_ClearCalibration failed.", LogLevel::ERROR, DeviceType::GUIDER);
        return false; // timeout
    }
    else
    {
        Logger::Log("call_phd_ClearCalibration success.", LogLevel::INFO, DeviceType::GUIDER);
        return true;
    }
}
uint32_t MainWindow::call_phd_StarClick(int x, int y)
{
    Logger::Log("call_phd_StarClick start ...", LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("call_phd_StarClick | x:" + std::to_string(x) + ", y:" + std::to_string(y), LogLevel::INFO, DeviceType::MAIN);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_StarClick | shared memory not ready", LogLevel::ERROR, DeviceType::MAIN);
        return false;
    }

    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x0f;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    unsigned char addr = 0;
    memcpy(sharedmemory_phd + baseAddress + addr, &x, sizeof(int));
    addr = addr + sizeof(int);
    memcpy(sharedmemory_phd + baseAddress + addr, &y, sizeof(int));
    addr = addr + sizeof(int);

    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    }
    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_StarClick | timeout", LogLevel::ERROR, DeviceType::MAIN);
        Logger::Log("call_phd_StarClick failed.", LogLevel::ERROR, DeviceType::MAIN);
        return false; // timeout
    }
    else
    {
        Logger::Log("call_phd_StarClick success.", LogLevel::INFO, DeviceType::MAIN);
        return true;
    }
}

uint32_t MainWindow::call_phd_FocalLength(int FocalLength)
{
    Logger::Log("call_phd_FocalLength start ...", LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("call_phd_FocalLength | FocalLength:" + std::to_string(FocalLength), LogLevel::INFO, DeviceType::MAIN);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_FocalLength | shared memory not ready", LogLevel::ERROR, DeviceType::MAIN);
        return false;
    }

    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x10;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    unsigned char addr = 0;
    memcpy(sharedmemory_phd + baseAddress + addr, &FocalLength, sizeof(int));
    addr = addr + sizeof(int);

    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    }
    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_FocalLength | timeout", LogLevel::ERROR, DeviceType::MAIN);
        Logger::Log("call_phd_FocalLength failed.", LogLevel::ERROR, DeviceType::MAIN);
        return false; // timeout
    }
    else
    {
        Logger::Log("call_phd_FocalLength success.", LogLevel::INFO, DeviceType::MAIN);
        return true;
    }
}

uint32_t MainWindow::call_phd_MultiStarGuider(bool isMultiStar)
{
    Logger::Log("call_phd_MultiStarGuider start ...", LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("call_phd_MultiStarGuider | isMultiStar:" + std::to_string(isMultiStar), LogLevel::INFO, DeviceType::MAIN);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_MultiStarGuider | shared memory not ready", LogLevel::ERROR, DeviceType::MAIN);
        return false;
    }

    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x11;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    unsigned char addr = 0;
    memcpy(sharedmemory_phd + baseAddress + addr, &isMultiStar, sizeof(bool));
    addr = addr + sizeof(bool);

    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    }
    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_MultiStarGuider | timeout", LogLevel::ERROR, DeviceType::MAIN);
        Logger::Log("call_phd_MultiStarGuider failed.", LogLevel::ERROR, DeviceType::MAIN);
        return false; // timeout
    }
    else
    {
        Logger::Log("call_phd_MultiStarGuider success.", LogLevel::INFO, DeviceType::MAIN);
        return true;
    }
}

uint32_t MainWindow::call_phd_CameraPixelSize(double PixelSize)
{
    Logger::Log("call_phd_CameraPixelSize start ...", LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("call_phd_CameraPixelSize | PixelSize:" + std::to_string(PixelSize), LogLevel::INFO, DeviceType::MAIN);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_CameraPixelSize | shared memory not ready", LogLevel::ERROR, DeviceType::MAIN);
        return false;
    }

    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x12;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    unsigned char addr = 0;
    memcpy(sharedmemory_phd + baseAddress + addr, &PixelSize, sizeof(double));
    addr = addr + sizeof(double);

    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    }
    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_CameraPixelSize | timeout", LogLevel::ERROR, DeviceType::MAIN);
        Logger::Log("call_phd_CameraPixelSize failed.", LogLevel::ERROR, DeviceType::MAIN);
        return false; // timeout
    }
    else
    {
        Logger::Log("call_phd_CameraPixelSize success.", LogLevel::INFO, DeviceType::MAIN);
        return true;
    }
}

uint32_t MainWindow::call_phd_CameraGain(int Gain)
{
    Logger::Log("call_phd_CameraGain start ...", LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("call_phd_CameraGain | Gain:" + std::to_string(Gain), LogLevel::INFO, DeviceType::MAIN);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_CameraGain | shared memory not ready", LogLevel::ERROR, DeviceType::MAIN);
        return false;
    }

    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x13;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    unsigned char addr = 0;
    memcpy(sharedmemory_phd + baseAddress + addr, &Gain, sizeof(int));
    addr = addr + sizeof(int);

    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    }
    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_CameraGain | timeout", LogLevel::ERROR, DeviceType::MAIN);
        Logger::Log("call_phd_CameraGain failed.", LogLevel::ERROR, DeviceType::MAIN);
        return false; // timeout
    }
    else
    {
        Logger::Log("call_phd_CameraGain success.", LogLevel::INFO, DeviceType::MAIN);
        return true;
    }
}

uint32_t MainWindow::call_phd_CalibrationDuration(int StepSize)
{
    Logger::Log("call_phd_CalibrationDuration start ...", LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("call_phd_CalibrationDuration | StepSize:" + std::to_string(StepSize), LogLevel::INFO, DeviceType::MAIN);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_CalibrationDuration | shared memory not ready", LogLevel::ERROR, DeviceType::MAIN);
        return false;
    }

    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x14;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    unsigned char addr = 0;
    memcpy(sharedmemory_phd + baseAddress + addr, &StepSize, sizeof(int));
    addr = addr + sizeof(int);

    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    }
    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_CalibrationDuration | timeout", LogLevel::ERROR, DeviceType::MAIN);
        Logger::Log("call_phd_CalibrationDuration failed.", LogLevel::ERROR, DeviceType::MAIN);
        return false; // timeout
    }
    else
    {
        Logger::Log("call_phd_CalibrationDuration success.", LogLevel::INFO, DeviceType::MAIN);
        return true;
    }
}

uint32_t MainWindow::call_phd_RaAggression(int Aggression)
{
    Logger::Log("call_phd_RaAggression start ...", LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("call_phd_RaAggression | Aggression:" + std::to_string(Aggression), LogLevel::INFO, DeviceType::MAIN);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_RaAggression | shared memory not ready", LogLevel::ERROR, DeviceType::MAIN);
        return false;
    }

    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x15;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    unsigned char addr = 0;
    memcpy(sharedmemory_phd + baseAddress + addr, &Aggression, sizeof(int));
    addr = addr + sizeof(int);

    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    }
    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_RaAggression | timeout", LogLevel::ERROR, DeviceType::MAIN);
        Logger::Log("call_phd_RaAggression failed.", LogLevel::ERROR, DeviceType::MAIN);
        return false; // timeout
    }
    else
    {
        Logger::Log("call_phd_RaAggression success.", LogLevel::INFO, DeviceType::MAIN);
        return true;
    }
}

uint32_t MainWindow::call_phd_DecAggression(int Aggression)
{
    Logger::Log("call_phd_DecAggression start ...", LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("call_phd_DecAggression | Aggression:" + std::to_string(Aggression), LogLevel::INFO, DeviceType::MAIN);
    // 修复：检查共享内存指针有效性
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("call_phd_DecAggression | shared memory not ready", LogLevel::ERROR, DeviceType::MAIN);
        return false;
    }

    unsigned int vendcommand;
    unsigned int baseAddress;

    bzero(sharedmemory_phd, 1024); // 共享内存清空

    baseAddress = 0x03;
    vendcommand = 0x16;

    sharedmemory_phd[1] = Tools::MSB(vendcommand);
    sharedmemory_phd[2] = Tools::LSB(vendcommand);

    sharedmemory_phd[0] = 0x01; // enable command

    unsigned char addr = 0;
    memcpy(sharedmemory_phd + baseAddress + addr, &Aggression, sizeof(int));
    addr = addr + sizeof(int);

    QElapsedTimer t;
    t.start();

    while (sharedmemory_phd[0] == 0x01 && t.elapsed() < 500)
    {
        // QCoreApplication::processEvents();
    }
    if (t.elapsed() >= 500)
    {
        Logger::Log("call_phd_DecAggression | timeout", LogLevel::ERROR, DeviceType::MAIN);
        Logger::Log("call_phd_DecAggression failed.", LogLevel::ERROR, DeviceType::MAIN);
        return false; // timeout
    }
    else
    {
        Logger::Log("call_phd_DecAggression success.", LogLevel::INFO, DeviceType::MAIN);
        return true;
    }
}

// ===================== 读取端：共享内存像素解码（完整实现） =====================
// 说明：
// 1) 保持你原有的共享内存布局不变：
//    - [0 .. 1023]  : 预留区（新增 V2 头 ShmHdrV2 放在这里，兼容旧读端）
//    - [1024 ..]    : 你原有的头/状态/导星数据（currentPHDSizeX/Y、bitDepth、各项 guide 数据等）
//    - [2047]       : 帧完成标志位（0x01=写入中，0x02=完成，0x00=已读）
//    - [2048 .. end]: 像素数据（RAW / RLE 压缩 / NEAREST 缩放）
// 2) 本实现自动识别新头（若存在），按 coding 解码；若无新头，则回退旧逻辑。
// 3) 不改变你已有的“读取的内容”（导星/状态字段），仅在像素拷贝前做安全边界检查与解码。


// ===== RLE 解压（速度优先的简单实现）=====
static bool rle_decompress_8(const uint8_t* src, size_t n, uint8_t* dst, size_t outPixels) {
    size_t si=0, di=0;
    while (si+1 <= n && di < outPixels) {
        if (si + 1 > n) return false;
        uint8_t v = src[si++];
        if (si >= n) return false;
        uint8_t run = src[si++];
        if ((size_t)di + run > outPixels) return false;
        std::memset(dst + di, v, run);
        di += run;
    }
    return di == outPixels;
}

static bool rle_decompress_16(const uint8_t* src, size_t n, uint16_t* dst, size_t outPixels) {
    size_t si=0, di=0;
    while (si+4 <= n && di < outPixels) {
        uint16_t v, run;
        std::memcpy(&v,   src+si, 2); si += 2;
        std::memcpy(&run, src+si, 2); si += 2;
        if ((size_t)di + run > outPixels) return false;
        for (uint16_t k=0;k<run;++k) dst[di++] = v;
    }
    return di == outPixels;
}

// ====== 完整的读取函数（在你的 MainWindow 类内）=====
void MainWindow::ShowPHDdata()
{
    // 修复：增强共享内存指针安全检查
    // 早退：共享内存可用且帧完成
    if (!sharedmemory_phd || sharedmemory_phd == (char*)-1) {
        Logger::Log("ShowPHDdata | shared memory not ready", LogLevel::ERROR, DeviceType::GUIDER);
        return;
    }
    
    // 修复：验证共享内存大小，确保kFlagOff在有效范围内
    const size_t total_size = (size_t)BUFSZ;
    if (kFlagOff >= total_size) {
        Logger::Log("ShowPHDdata | kFlagOff out of bounds", LogLevel::ERROR, DeviceType::GUIDER);
        return;
    }
    
    if (sharedmemory_phd[kFlagOff] != 0x02) {
        // 没有新帧
        return;
    }

    // ---------- 读图像原始头（与旧协议完全一致） ----------
    unsigned int currentPHDSizeX = 1;
    unsigned int currentPHDSizeY = 1;
    unsigned int bitDepth        = 8;

    unsigned int mem_offset = kHeaderOff;

    auto ensure = [&](size_t need) -> bool {
        // 头部区域必须在 payload 前结束
        return (mem_offset + need <= kPayloadOff && mem_offset + need <= total_size);
    };

    if (!ensure(sizeof(unsigned int))) { sharedmemory_phd[kFlagOff]=0x00; return; }
    std::memcpy(&currentPHDSizeX, sharedmemory_phd + mem_offset, sizeof(unsigned int));
    mem_offset += sizeof(unsigned int);

    if (!ensure(sizeof(unsigned int))) { sharedmemory_phd[kFlagOff]=0x00; return; }
    std::memcpy(&currentPHDSizeY, sharedmemory_phd + mem_offset, sizeof(unsigned int));
    mem_offset += sizeof(unsigned int);

    if (!ensure(sizeof(unsigned char))) { sharedmemory_phd[kFlagOff]=0x00; return; }
    std::memcpy(&bitDepth, sharedmemory_phd + mem_offset, sizeof(unsigned char));
    mem_offset += sizeof(unsigned char);

    if (!(bitDepth == 8 || bitDepth == 16)) {
        Logger::Log("ShowPHDdata | invalid bitDepth: " + std::to_string(bitDepth), LogLevel::WARNING, DeviceType::GUIDER);
        sharedmemory_phd[kFlagOff] = 0x00;
        return;
    }

    /* ------------------------------  新增：先读 V2 头，决定本帧尺寸  ------------------------------ */
    ShmHdrV2 v2{}; 
    bool hasV2 = false;
    if (total_size >= sizeof(ShmHdrV2)) {
        std::memcpy(&v2, sharedmemory_phd, sizeof(ShmHdrV2));
        hasV2 = (v2.magic == SHM_MAGIC && v2.version == SHM_VER);
    }

    // 本帧用于 UI/WS 的“实际尺寸/位深”
    uint32_t dispW = hasV2 ? v2.outW : currentPHDSizeX;
    uint32_t dispH = hasV2 ? v2.outH : currentPHDSizeY;
    uint16_t useDepth = hasV2 ? v2.bitDepth : (uint16_t)bitDepth;

    // 合法性兜底
    if (dispW == 0 || dispH == 0 || !(useDepth==8 || useDepth==16)) {
        // 回退到旧头
        hasV2 = false;
        dispW = currentPHDSizeX;
        dispH = currentPHDSizeY;
        useDepth = (uint16_t)bitDepth;
    }

    // 记录原始/输出尺寸与缩放倍数，供坐标换算使用
    glPHD_OrigImageSizeX = hasV2 ? (int)v2.origW : (int)currentPHDSizeX;
    glPHD_OrigImageSizeY = hasV2 ? (int)v2.origH : (int)currentPHDSizeY;
    glPHD_OutImageSizeX  = (int)dispW;
    glPHD_OutImageSizeY  = (int)dispH;
    {
        double sx = (glPHD_OutImageSizeX  > 0) ? (double)glPHD_OrigImageSizeX / (double)glPHD_OutImageSizeX  : 1.0;
        double sy = (glPHD_OutImageSizeY  > 0) ? (double)glPHD_OrigImageSizeY / (double)glPHD_OutImageSizeY  : 1.0;
        int s = (int)std::lround((sx + sy) * 0.5);
        if (s < 1) s = 1;
        glPHD_ImageScale = s;
    }

    // ---------- 跳过你原有的 3 个 int 字段（sdk_*） ----------
    if (!ensure(sizeof(int))) { sharedmemory_phd[kFlagOff]=0x00; return; }  mem_offset += sizeof(int);
    if (!ensure(sizeof(int))) { sharedmemory_phd[kFlagOff]=0x00; return; }  mem_offset += sizeof(int);
    if (!ensure(sizeof(int))) { sharedmemory_phd[kFlagOff]=0x00; return; }  mem_offset += sizeof(int);

    // ---------- 读取导星/状态数据（保持不变，不缺少） ----------
    unsigned int guideDataIndicatorAddress = (unsigned int)mem_offset;
    if (!ensure(sizeof(unsigned char))) { sharedmemory_phd[kFlagOff]=0x00; return; }
    unsigned char guideDataIndicator = *(unsigned char*)(sharedmemory_phd + mem_offset);
    mem_offset += sizeof(unsigned char);

    double dRa=0, dDec=0, SNR=0, MASS=0, RMSErrorX=0, RMSErrorY=0, RMSErrorTotal=0, PixelRatio=1;
    int RADUR=0, DECDUR=0; char RADIR=0, DECDIR=0; bool StarLostAlert=false, InGuiding=false;

    auto safe_copy = [&](void* dst, size_t n) -> bool {
        if (!ensure(n)) { sharedmemory_phd[kFlagOff]=0x00; return false; }
        std::memcpy(dst, sharedmemory_phd + mem_offset, n);
        mem_offset += n;
        return true;
    };

    if (!safe_copy(&dRa, sizeof(double))) return;
    if (!safe_copy(&dDec, sizeof(double))) return;
    if (!safe_copy(&SNR, sizeof(double))) return;
    if (!safe_copy(&MASS, sizeof(double))) return;
    if (!safe_copy(&RADUR, sizeof(int))) return;
    if (!safe_copy(&DECDUR, sizeof(int))) return;
    if (!safe_copy(&RADIR, sizeof(char))) return;
    if (!safe_copy(&DECDIR, sizeof(char))) return;
    if (!safe_copy(&RMSErrorX, sizeof(double))) return;
    if (!safe_copy(&RMSErrorY, sizeof(double))) return;
    if (!safe_copy(&RMSErrorTotal, sizeof(double))) return;
    if (!safe_copy(&PixelRatio, sizeof(double))) return;
    if (!safe_copy(&StarLostAlert, sizeof(bool))) return;
    if (!safe_copy(&InGuiding, sizeof(bool))) return;

    // 你的 1024+200 区域：锁星、十字、MultiStar（保持不变）
    mem_offset = kHeaderOff + 200;
    auto ensure_at = [&](size_t off, size_t n)->bool {
        return (off + n <= kPayloadOff && off + n <= total_size);
    };

    bool isSelected=false, showLockedCross=false;
    double StarX=0, StarY=0, LockedPositionX=0, LockedPositionY=0;
    unsigned char MultiStarNumber=0;
    unsigned short MultiStarX[32]={0}, MultiStarY[32]={0};

    if (!ensure_at(mem_offset, sizeof(bool))) { sharedmemory_phd[kFlagOff]=0x00; return; }
    std::memcpy(&isSelected, sharedmemory_phd + mem_offset, sizeof(bool)); mem_offset += sizeof(bool);

    if (!ensure_at(mem_offset, sizeof(double))) { sharedmemory_phd[kFlagOff]=0x00; return; }
    std::memcpy(&StarX, sharedmemory_phd + mem_offset, sizeof(double)); mem_offset += sizeof(double);

    if (!ensure_at(mem_offset, sizeof(double))) { sharedmemory_phd[kFlagOff]=0x00; return; }
    std::memcpy(&StarY, sharedmemory_phd + mem_offset, sizeof(double)); mem_offset += sizeof(double);

    if (!ensure_at(mem_offset, sizeof(bool))) { sharedmemory_phd[kFlagOff]=0x00; return; }
    std::memcpy(&showLockedCross, sharedmemory_phd + mem_offset, sizeof(bool)); mem_offset += sizeof(bool);

    if (!ensure_at(mem_offset, sizeof(double))) { sharedmemory_phd[kFlagOff]=0x00; return; }
    std::memcpy(&LockedPositionX, sharedmemory_phd + mem_offset, sizeof(double)); mem_offset += sizeof(double);

    if (!ensure_at(mem_offset, sizeof(double))) { sharedmemory_phd[kFlagOff]=0x00; return; }
    std::memcpy(&LockedPositionY, sharedmemory_phd + mem_offset, sizeof(double)); mem_offset += sizeof(double);

    if (!ensure_at(mem_offset, sizeof(unsigned char))) { sharedmemory_phd[kFlagOff]=0x00; return; }
    std::memcpy(&MultiStarNumber, sharedmemory_phd + mem_offset, sizeof(unsigned char)); mem_offset += sizeof(unsigned char);
    MultiStarNumber = std::min<unsigned char>(MultiStarNumber, 32);

    if (!ensure_at(mem_offset, sizeof(MultiStarX))) { sharedmemory_phd[kFlagOff]=0x00; return; }
    std::memcpy(MultiStarX, sharedmemory_phd + mem_offset, sizeof(MultiStarX)); mem_offset += sizeof(MultiStarX);

    if (!ensure_at(mem_offset, sizeof(MultiStarY))) { sharedmemory_phd[kFlagOff]=0x00; return; }
    std::memcpy(MultiStarY, sharedmemory_phd + mem_offset, sizeof(MultiStarY)); mem_offset += sizeof(MultiStarY);

    // 清除导星数据指示位（保持你的行为）
    sharedmemory_phd[guideDataIndicatorAddress] = 0x00;

    // ---------- 将导星/锁星信息分发到 UI/WS（保持你的逻辑） ----------
    glPHD_isSelected         = isSelected;
    glPHD_StarX              = StarX;
    glPHD_StarY              = StarY;
    glPHD_CurrentImageSizeX  = dispW;   // UI 显示尺寸（合并/缩放后）
    glPHD_CurrentImageSizeY  = dispH;   // UI 显示尺寸（合并/缩放后）
    glPHD_LockPositionX      = LockedPositionX;
    glPHD_LockPositionY      = LockedPositionY;
    glPHD_ShowLockCross      = showLockedCross;

    glPHD_Stars.clear();
    emit wsThread->sendMessageToClient("ClearPHD2MultiStars");
    const double mapRatioX = (glPHD_OrigImageSizeX > 0) ? (double)glPHD_OutImageSizeX / (double)glPHD_OrigImageSizeX : 1.0;
    const double mapRatioY = (glPHD_OrigImageSizeY > 0) ? (double)glPHD_OutImageSizeY / (double)glPHD_OrigImageSizeY : 1.0;
    for (int i = 1; i < MultiStarNumber; i++) {
        if (i > 12) break;
        int outX = (int)std::lround(MultiStarX[i] * mapRatioX);
        int outY = (int)std::lround(MultiStarY[i] * mapRatioY);
        if (outX < 0) outX = 0;
        if (outY < 0) outY = 0;
        if (outX >= glPHD_OutImageSizeX) outX = glPHD_OutImageSizeX - 1;
        if (outY >= glPHD_OutImageSizeY) outY = glPHD_OutImageSizeY - 1;
        QPoint p; p.setX(outX); p.setY(outY);
        glPHD_Stars.push_back(p);
        emit wsThread->sendMessageToClient(
            "PHD2MultiStarsPosition:" + QString::number(glPHD_CurrentImageSizeX) + ":" +
            QString::number(glPHD_CurrentImageSizeY) + ":" +
            QString::number(outX) + ":" + QString::number(outY));
    }

    if (glPHD_isSelected) {
        emit wsThread->sendMessageToClient("PHD2StarBoxView:true");
        int outStarX = (int)std::lround(glPHD_StarX * mapRatioX);
        int outStarY = (int)std::lround(glPHD_StarY * mapRatioY);
        if (outStarX < 0) outStarX = 0;
        if (outStarY < 0) outStarY = 0;
        if (outStarX >= glPHD_OutImageSizeX) outStarX = glPHD_OutImageSizeX - 1;
        if (outStarY >= glPHD_OutImageSizeY) outStarY = glPHD_OutImageSizeY - 1;
        emit wsThread->sendMessageToClient(
            "PHD2StarBoxPosition:" + QString::number(glPHD_CurrentImageSizeX) + ":" +
            QString::number(glPHD_CurrentImageSizeY) + ":" +
            QString::number(outStarX) + ":" + QString::number(outStarY));
    } else {
        emit wsThread->sendMessageToClient("PHD2StarBoxView:false");
    }

    if (glPHD_ShowLockCross) {
        emit wsThread->sendMessageToClient("PHD2StarCrossView:true");
        int outLockX = (int)std::lround(glPHD_LockPositionX * mapRatioX);
        int outLockY = (int)std::lround(glPHD_LockPositionY * mapRatioY);
        if (outLockX < 0) outLockX = 0;
        if (outLockY < 0) outLockY = 0;
        if (outLockX >= glPHD_OutImageSizeX) outLockX = glPHD_OutImageSizeX - 1;
        if (outLockY >= glPHD_OutImageSizeY) outLockY = glPHD_OutImageSizeY - 1;
        emit wsThread->sendMessageToClient(
            "PHD2StarCrossPosition:" + QString::number(glPHD_CurrentImageSizeX) + ":" +
            QString::number(glPHD_CurrentImageSizeY) + ":" +
            QString::number(outLockX) + ":" + QString::number(outLockY));
    } else {
        emit wsThread->sendMessageToClient("PHD2StarCrossView:false");
    }

    // ---------- 导星状态/曲线（保持你的逻辑） ----------
    if (sharedmemory_phd[kFlagOff] == 0x02 && bitDepth > 0 && currentPHDSizeX > 0 && currentPHDSizeY > 0) {
        unsigned char phdstatu;
        call_phd_checkStatus(phdstatu);

        Logger::Log("ShowPHDdata | dRa:" + std::to_string(dRa) + ", dDec:" + std::to_string(dDec),
                    LogLevel::DEBUG, DeviceType::GUIDER);

        if (dRa != 0 || dDec != 0) {
            QPointF tmp; tmp.setX(-dRa * PixelRatio); tmp.setY(dDec * PixelRatio);
            glPHD_rmsdate.append(tmp);
            emit wsThread->sendMessageToClient("AddScatterChartData:" +
                QString::number(-dRa * PixelRatio) + ":" + QString::number(-dDec * PixelRatio));

            if (InGuiding) {
                emit wsThread->sendMessageToClient("GuiderStatus:InGuiding");
                emit wsThread->sendMessageToClient("GuiderUpdateStatus:1");
            } else {
                emit wsThread->sendMessageToClient("GuiderStatus:InCalibration");
                emit wsThread->sendMessageToClient("GuiderUpdateStatus:1");
            }

            if (StarLostAlert) {
                Logger::Log("ShowPHDdata | send GuiderStatus:StarLostAlert",
                            LogLevel::DEBUG, DeviceType::GUIDER);
                emit wsThread->sendMessageToClient("GuiderStatus:StarLostAlert");
                emit wsThread->sendMessageToClient("GuiderUpdateStatus:2");
            }

            emit wsThread->sendMessageToClient("AddRMSErrorData:" +
                QString::number(RMSErrorX, 'f', 3) + ":" + QString::number(RMSErrorX, 'f', 3));
        }

        for (int i = 0; i < glPHD_rmsdate.size(); i++) {
            if (i == glPHD_rmsdate.size() - 1) {
                emit wsThread->sendMessageToClient("AddLineChartData:" + QString::number(i) + ":" +
                    QString::number(glPHD_rmsdate[i].x()) + ":" + QString::number(glPHD_rmsdate[i].y()));
                if (i > 50)
                    emit wsThread->sendMessageToClient("SetLineChartRange:" + QString::number(i - 50) + ":" + QString::number(i));
                else
                    emit wsThread->sendMessageToClient("SetLineChartRange:0:50");
            }
        }
    }

    // ===================== 像素数据读取 / 解码 =====================
    const size_t payload_cap_bytes = (total_size > kPayloadOff) ? (total_size - kPayloadOff) : 0;
    if (payload_cap_bytes == 0) { sharedmemory_phd[kFlagOff] = 0x00; return; }

    cv::Mat PHDImg;
    std::unique_ptr<uint8_t[]> buf;

    if (hasV2) {
        Logger::Log("V2 hdr: coding=" + std::to_string(v2.coding) +
                    " out=" + std::to_string(v2.outW) + "x" + std::to_string(v2.outH) +
                    " depth=" + std::to_string(v2.bitDepth) +
                    " payload=" + std::to_string(v2.payloadSize),
                    LogLevel::DEBUG, DeviceType::GUIDER);

        const uint16_t coding   = v2.coding;
        const uint16_t v2Depth  = v2.bitDepth;
        const size_t   bpp      = (v2Depth == 16) ? 2 : 1;
        const uint32_t outW     = v2.outW;
        const uint32_t outH     = v2.outH;
        const size_t   outPix   = (size_t)outW * (size_t)outH;
        const size_t   need     = outPix * bpp;
        const size_t   payLen   = (size_t)v2.payloadSize;

        if (!(v2Depth==8 || v2Depth==16) || outW==0 || outH==0 || payLen > payload_cap_bytes) {
            // 头异常，退回旧逻辑
            hasV2 = false;
        } else {
            const uint8_t* payload = (const uint8_t*)(sharedmemory_phd + kPayloadOff);

            if (coding == CODING_RAW || coding == CODING_NEAREST) {
                if (need > payload_cap_bytes) { sharedmemory_phd[kFlagOff] = 0x00; return; }
                buf.reset(new uint8_t[need]);
                std::memcpy(buf.get(), payload, need);
                if (v2Depth == 16) PHDImg.create(outH, outW, CV_16UC1);
                else               PHDImg.create(outH, outW, CV_8UC1);
                PHDImg.data = buf.get();
            } else if (coding == CODING_RLE) {
                buf.reset(new uint8_t[need]);
                bool ok = (v2Depth==8)
                    ? rle_decompress_8(payload, payLen, buf.get(), outPix)
                    : rle_decompress_16(payload, payLen, (uint16_t*)buf.get(), outPix);
                if (!ok) { sharedmemory_phd[kFlagOff] = 0x00; return; }
                if (v2Depth == 16) PHDImg.create(outH, outW, CV_16UC1);
                else               PHDImg.create(outH, outW, CV_8UC1);
                PHDImg.data = buf.get();
            } else {
                hasV2 = false;
            }
        }
    }

    if (!hasV2) {
        Logger::Log("Legacy path (no V2 header)", LogLevel::DEBUG, DeviceType::GUIDER);
        const size_t need = (size_t)currentPHDSizeX * (size_t)currentPHDSizeY * (bitDepth / 8);
        if (need == 0 || need > payload_cap_bytes) {
            Logger::Log("ShowPHDdata | legacy frame too large or zero", LogLevel::WARNING, DeviceType::GUIDER);
            sharedmemory_phd[kFlagOff] = 0x00;
            return;
        }
        buf.reset(new uint8_t[need]);
        std::memcpy(buf.get(), sharedmemory_phd + kPayloadOff, need);
        if (bitDepth == 16) PHDImg.create(currentPHDSizeY, currentPHDSizeX, CV_16UC1);
        else                PHDImg.create(currentPHDSizeY, currentPHDSizeX, CV_8UC1);
        PHDImg.data = buf.get();
    }

    // 像素完整拷贝/解码完成后再清 2047 标志，避免半帧被清
    sharedmemory_phd[kFlagOff] = 0x00;

    // ===== 你的后处理：拉伸/保存/显示（保持不变）=====
    uint16_t B = 0;
    uint16_t W = (PHDImg.depth() == CV_8U) ? 255 : 65535;  // 根据图像位深度设置默认最大值

    cv::Mat image_raw8;
    image_raw8.create(PHDImg.rows, PHDImg.cols, CV_8UC1);

    if (AutoStretch == true) {
        Tools::GetAutoStretch(PHDImg, 0, B, W);
    } else {
        B = 0;
        W = (PHDImg.depth() == CV_8U) ? 255 : 65535;  // 根据图像位深度设置默认最大值
    }
    Tools::Bit16To8_Stretch(PHDImg, image_raw8, B, W);
    saveGuiderImageAsJPG(image_raw8);

}



void MainWindow::ControlGuide(int Direction, int Duration)
{
    Logger::Log("ControlGuide start ...", LogLevel::INFO, DeviceType::GUIDER);
    switch (Direction)
    {
    case 1:
    {
        if (dpMount != NULL)
        {
            if (isMeridianFlipped)
            {
                Logger::Log("ControlGuide | setTelescopeGuideNS Direction:0, Duration:" + std::to_string(Duration), LogLevel::INFO, DeviceType::GUIDER);
                indi_Client->setTelescopeGuideNS(dpMount, 0, Duration);
            }
            else
            {
                Logger::Log("ControlGuide | setTelescopeGuideNS Direction:" + std::to_string(Direction) + ", Duration:" + std::to_string(Duration), LogLevel::INFO, DeviceType::GUIDER);
                indi_Client->setTelescopeGuideNS(dpMount, Direction, Duration);
            }
        }
        break;
    }
    case 0:
    {
        if (dpMount != NULL)
        {
            if (isMeridianFlipped)
            {
                Logger::Log("ControlGuide | setTelescopeGuideNS Direction:1, Duration:" + std::to_string(Duration), LogLevel::INFO, DeviceType::GUIDER);
                indi_Client->setTelescopeGuideNS(dpMount, 1, Duration);
            }
            else
            {
                Logger::Log("ControlGuide | setTelescopeGuideNS Direction:" + std::to_string(Direction) + ", Duration:" + std::to_string(Duration), LogLevel::INFO, DeviceType::GUIDER);
                indi_Client->setTelescopeGuideNS(dpMount, Direction, Duration);
            }
        }
        break;
    }
    case 2:
    {
        if (dpMount != NULL)
        {
            Logger::Log("ControlGuide | setTelescopeGuideWE Direction:" + std::to_string(Direction) + ", Duration:" + std::to_string(Duration), LogLevel::INFO, DeviceType::GUIDER);
            indi_Client->setTelescopeGuideWE(dpMount, Direction, Duration);
        }
        break;
    }
    case 3:
    {
        if (dpMount != NULL)
        {
            Logger::Log("ControlGuide | setTelescopeGuideWE Direction:" + std::to_string(Direction) + ", Duration:" + std::to_string(Duration), LogLevel::INFO, DeviceType::GUIDER);
            indi_Client->setTelescopeGuideWE(dpMount, Direction, Duration);
        }
        break;
    }
    default:
        break; //
    }
    Logger::Log("ControlGuide finish!", LogLevel::INFO, DeviceType::GUIDER);
}

void MainWindow::getTimeNow(int index)
{
    // 获取当前时间点
    auto now = std::chrono::system_clock::now();

    // 将当前时间点转换为毫秒
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();

    // 将毫秒时间戳转换为时间类型（std::time_t）
    std::time_t time_now = ms / 1000; // 将毫秒转换为秒

    // 使用 std::strftime 函数将时间格式化为字符串
    char buffer[80];
    std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S",
                  std::localtime(&time_now));

    // 添加毫秒部分
    std::string formatted_time = buffer + std::to_string(ms % 1000);

    // 打印带有当前时间的输出
    // std::cout << "TIME(ms): " << formatted_time << "," << index << std::endl;
}

void MainWindow::onPHDControlGuideTimeout()
{
    // Logger::Log("PHD2 Control Guide is Timeout !", LogLevel::DEBUG, DeviceType::MAIN);
    GetPHD2ControlInstruct();
}

void MainWindow::GetPHD2ControlInstruct()
{
    // Logger::Log("GetPHD2ControlInstruct start ...", LogLevel::DEBUG, DeviceType::MAIN);
    std::lock_guard<std::mutex> lock(receiveMutex);

    unsigned int mem_offset;

    int sdk_direction = 0;
    int sdk_duration = 0;
    int sdk_num = 0;
    int zero = 0;
    mem_offset = 1024;

    mem_offset = mem_offset + sizeof(unsigned int);
    mem_offset = mem_offset + sizeof(unsigned int);
    mem_offset = mem_offset + sizeof(unsigned char);

    int ControlInstruct = 0;

    memcpy(&ControlInstruct, sharedmemory_phd + mem_offset, sizeof(int));
    // Logger::Log("GetPHD2ControlInstruct | get ControlInstruct:" + std::to_string(ControlInstruct), LogLevel::DEBUG, DeviceType::MAIN);
    int mem_offset_sdk_num = mem_offset;
    mem_offset = mem_offset + sizeof(int);

    sdk_num = (ControlInstruct >> 24) & 0xFFF;       // 取前12位
    sdk_direction = (ControlInstruct >> 12) & 0xFFF; // 取中间12位
    sdk_duration = ControlInstruct & 0xFFF;          // 取后12位

    if (sdk_num != 0)
    {
        getTimeNow(sdk_num);
        // Logger::Log("GetPHD2ControlInstruct | PHD2ControlTelescope:" + std::to_string(sdk_num) + "," + std::to_string(sdk_direction) + "," + std::to_string(sdk_duration), LogLevel::DEBUG, DeviceType::MAIN);
    }
    if (sdk_duration != 0)
    {
        MainWindow::ControlGuide(sdk_direction, sdk_duration);

        memcpy(sharedmemory_phd + mem_offset_sdk_num, &zero, sizeof(int));
        // Logger::Log("GetPHD2ControlInstruct | set ControlInstruct to 0", LogLevel::DEBUG, DeviceType::MAIN);
        call_phd_ChackControlStatus(sdk_num); // set pFrame->ControlStatus = 0;
    }
    // Logger::Log("GetPHD2ControlInstruct finish!", LogLevel::DEBUG, DeviceType::MAIN);
}


void MainWindow::HandleFocuserMovementDataPeriodically()
{
    if (!isFocusMoveDone)
    {
        focusMoveTimer->stop();
        return;
    }
    if (dpFocuser == NULL)
    {
        focusMoveTimer->stop();
        return;
    }
    CurrentPosition = FocuserControl_getPosition();

    if (CurrentPosition == INT_MIN)
    {
        Logger::Log("HandleFocuserMovementDataPeriodically | get current position failed!", LogLevel::WARNING, DeviceType::FOCUSER);
    }
    else
    {
        // Logger::Log("HandleFocuserMovementDataPeriodically | Current Position: " + std::to_string(CurrentPosition), LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient("FocusPosition:" + QString::number(CurrentPosition) + ":" + QString::number(CurrentPosition));
    }
    // Logger::Log("HandleFocuserMovementDataPeriodically | 定时器判断当前位置: " + std::to_string(CurrentPosition), LogLevel::INFO, DeviceType::FOCUSER);
    bool isInward = this->currentDirection; // 假设您有一个成员变量来存储当前的移动方向
    if (isInward)
    {
        if (CurrentPosition == TargetPosition)
        {
            int steps = CurrentPosition - focuserMinPosition;
            if (steps > 60000)  // 特殊定义,单次移动距离不得大于60000步
            {
                steps = 60000;
                TargetPosition = CurrentPosition - steps;
            }
            else
            {
                TargetPosition = focuserMinPosition;
            }
            if (steps <= 0)
            {
                emit wsThread->sendMessageToClient("FocusMoveToLimit:The current position has moved to the inner limit and cannot move further. If you need to continue moving, please recalibrate the position of the servo.");
                return;
            }

            // steps = std::min(steps, focuserMaxPosition);
            // steps = std::max(steps, focuserMinPosition);
            indi_Client->setFocuserMoveDiretion(dpFocuser, isInward);
            indi_Client->moveFocuserSteps(dpFocuser, steps);

            Logger::Log("HandleFocuserMovementDataPeriodically | CurrentPosition: " + std::to_string(CurrentPosition) + " ,set move steps " + std::to_string(steps) + " ,backward inward", LogLevel::DEBUG, DeviceType::FOCUSER);
        }
        else
        {
            Logger::Log("Moving inward ... , CurrentPosition: " + std::to_string(CurrentPosition), LogLevel::DEBUG, DeviceType::FOCUSER);
        }
    }
    else if (!isInward)
    {
        if (TargetPosition == CurrentPosition)
        {
            int steps = focuserMaxPosition - CurrentPosition;
            if (steps > 60000)  // 特殊定义,单次移动距离不得大于60000步
            {
                steps = 60000;
                TargetPosition = CurrentPosition + steps;
            }
            else
            {
                TargetPosition = focuserMinPosition;
            }
            if (steps <= 0)
            {
                emit wsThread->sendMessageToClient("FocusMoveToLimit:The current position has moved to the outer limit and cannot move further. If you need to continue moving, please recalibrate the position of the servo.");
                return;
            }
            // steps = std::min(steps, focuserMaxPosition);
            // steps = std::max(steps, focuserMinPosition);
            indi_Client->setFocuserMoveDiretion(dpFocuser, isInward);
            indi_Client->moveFocuserSteps(dpFocuser, steps);
            Logger::Log("HandleFocuserMovementDataPeriodically | CurrentPosition: " + std::to_string(CurrentPosition) + " ,set move steps " + std::to_string(steps) + " ,backward outward", LogLevel::DEBUG, DeviceType::FOCUSER);
        }
        else
        {
            Logger::Log("Moving outward ... , CurrentPosition: " + std::to_string(CurrentPosition), LogLevel::INFO, DeviceType::FOCUSER);
        }
    }
    focusMoveEndTime -= 0.5;
    CheckFocuserMoveOrder();
    if (focusMoveEndTime <= 0)
    {
        FocuserControlStop();
    }
}

void MainWindow::FocuserControlMove(bool isInward)
{
    this->currentDirection = isInward; // 更新当前方向
    if (dpFocuser == NULL)
    {
        Logger::Log("FocuserControlMove | dpFocuser is NULL", LogLevel::WARNING, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient("FocusMoveDone:0:0");
        return;
    }
    focusMoveEndTime = 2;
    isFocusMoveDone = true;
    CurrentPosition = FocuserControl_getPosition();
    TargetPosition = CurrentPosition;
    startPosition = CurrentPosition;
    if (CurrentPosition >= focuserMaxPosition && !isInward)
    {
        emit wsThread->sendMessageToClient("FocusMoveToLimit:The current position has moved to the inner limit and cannot move further. If you need to continue moving, please recalibrate the position of the servo.");
        focusMoveTimer->stop();
        return;
    }
    else if (CurrentPosition <= focuserMinPosition && isInward)
    {
        emit wsThread->sendMessageToClient("FocusMoveToLimit:The current position has moved to the outer limit and cannot move further. If you need to continue moving, please recalibrate the position of the servo.");
        focusMoveTimer->stop();
        return;
    }
    HandleFocuserMovementDataPeriodically();
    focusMoveTimer->start(1000);
}

void MainWindow::FocuserControlStop(bool isClickMove)
{
    if (dpFocuser == NULL)
    {
        Logger::Log("focusMoveStop | dpFocuser is NULL", LogLevel::WARNING, DeviceType::FOCUSER);
        return;
    }
    Logger::Log("focusMoveStop | Stop Focuser Move", LogLevel::INFO, DeviceType::FOCUSER);
    CurrentPosition = FocuserControl_getPosition();
    // if (isClickMove)
    // {
    //     int steps = abs(CurrentPosition - startPosition);
    //     int time = 1;
    //     while (steps < 100 && time < 10)
    //     {
    //         CurrentPosition = FocuserControl_getPosition();
    //         steps = abs(CurrentPosition - startPosition); // 删除int，避免重复声明局部变量
    //         time++;
    //         usleep(100000); // 修改为0.1秒 (100,000微秒)
    //     }
    //     Logger::Log("focusMoveStop | Click Move Steps: " + std::to_string(steps), LogLevel::INFO, DeviceType::FOCUSER);
    // }
    if (dpFocuser != NULL)
    {
        indi_Client->abortFocuserMove(dpFocuser);
    }
    else
    {
        Logger::Log("focusMoveStop | dpFocuser is NULL", LogLevel::WARNING, DeviceType::FOCUSER);
    }

    if (focusMoveTimer->isActive())
    {
        focusMoveTimer->stop();
    }
    if (dpFocuser != NULL)
    {
        CurrentPosition = FocuserControl_getPosition();
    }
    else
    {
        CurrentPosition = 0;
    }
    isFocusMoveDone = false;
    emit wsThread->sendMessageToClient("FocusPosition:" + QString::number(CurrentPosition) + ":" + QString::number(CurrentPosition));
    if (updatePositionTimer != nullptr)
    {
        // 添加计时器以定期更新位置
        updatePositionTimer->stop();
        updatePositionTimer->deleteLater();
        updatePositionTimer = nullptr;
    }
    updatePositionTimer = new QTimer(this);
    updatePositionTimer->setInterval(1000); // 设置计时器间隔为1000毫秒
    updateCount = 0;                        // 初始化计数器

    connect(updatePositionTimer, &QTimer::timeout, [this]()
            {
        if (isFocusMoveDone || updateCount >= 3) {
            updatePositionTimer->stop();
            updatePositionTimer->deleteLater();
            updatePositionTimer = nullptr;
            Logger::Log("focusMoveStop | Timer manually released", LogLevel::INFO, DeviceType::FOCUSER);
            return;
        }
        CurrentPosition = FocuserControl_getPosition();
        emit wsThread->sendMessageToClient("FocusPosition:" + QString::number(CurrentPosition) + ":" + QString::number(CurrentPosition));
        Logger::Log("focusMoveStop | Current Focuser Position: " + std::to_string(CurrentPosition), LogLevel::INFO, DeviceType::FOCUSER);
        updateCount++; });

    updatePositionTimer->start(); // 启动计时器

    Logger::Log("focusMoveStop | Current Focuser Position: " + std::to_string(CurrentPosition), LogLevel::INFO, DeviceType::FOCUSER);
}

void MainWindow::CheckFocuserMoveOrder()
{
    emit wsThread->sendMessageToClient("getFocuserMoveState");
}

void MainWindow::FocuserControlMoveStep(bool isInward, int steps)
{
    // 记录开始移动焦点器的日志
    Logger::Log("FocuserControlMoveStep start ...", LogLevel::INFO, DeviceType::FOCUSER);
    if (isStepMoving)
    {
        Logger::Log("FocuserControlMoveStep | isStepMoving is true, return", LogLevel::INFO, DeviceType::FOCUSER);
        return;
    }
    if (dpFocuser != NULL)
    {
        // 防重入：已有一次步进移动在进行中时，先取消上一次，避免重复连接与计时器叠加
        cancelStepMoveIfAny();

        // 获取当前焦点器的位置
        CurrentPosition = FocuserControl_getPosition();

        // 根据移动方向计算目标位置
        if(isInward == false)
        {
            TargetPosition = CurrentPosition + steps;
        }
        else
        {
            TargetPosition = CurrentPosition - steps;
        }
        // 记录目标位置的日志
        Logger::Log("FocuserControlMoveStep | Target Position: " + std::to_string(TargetPosition), LogLevel::INFO, DeviceType::FOCUSER);

        // 设置焦点器的移动方向并执行移动
        if (TargetPosition > focuserMaxPosition)
        {
            TargetPosition = focuserMaxPosition;
        }
        else if (TargetPosition < focuserMinPosition)
        {
            TargetPosition = focuserMinPosition;
        }
        steps = std::abs(TargetPosition - CurrentPosition);
        if (steps <= 0 && !isInward)
        {
            emit wsThread->sendMessageToClient("FocusMoveToLimit:The current position has moved to the inner limit and cannot move further. If you need to continue moving, please recalibrate the position of the servo.");
            return;
        }
        else if (steps <= 0 && isInward)
        {
            emit wsThread->sendMessageToClient("FocusMoveToLimit:The current position has moved to the outer limit and cannot move further. If you need to continue moving, please recalibrate the position of the servo.");
            return;
        }
        // 标记占用，防止后续点击累加
        isStepMoving = true;
        stepMoveOutTime = 10;
        indi_Client->setFocuserMoveDiretion(dpFocuser, isInward);
        indi_Client->moveFocuserSteps(dpFocuser, steps);

        // 设置计时器为单次触发
        focusTimer.setSingleShot(true);
        
        // 先断开旧的连接，避免重复连接导致多次回调
        disconnect(&focusTimer, &QTimer::timeout, this, nullptr);

        // 连接定时回调，检查到位与刷位置
        connect(&focusTimer, &QTimer::timeout, this, [this]() {
            stepMoveOutTime--;
            CurrentPosition = FocuserControl_getPosition();
            emit wsThread->sendMessageToClient("FocusPosition:" + QString::number(CurrentPosition) + ":" + QString::number(CurrentPosition));
            if (CurrentPosition <= focuserMinPosition || CurrentPosition >= focuserMaxPosition || stepMoveOutTime <= 0 || CurrentPosition == TargetPosition) {
                focusTimer.stop();
                disconnect(&focusTimer, &QTimer::timeout, this, nullptr); // 断开连接，避免重复触发
                isStepMoving = false;
                Logger::Log("FocuserControlMoveStep | Focuser Move Complete!", LogLevel::INFO, DeviceType::FOCUSER);
                emit wsThread->sendMessageToClient("FocusMoveDone:" + QString::number(CurrentPosition));
            } else {
                focusTimer.start(100);
            }
        });
        
        // 启动定时器，开始检查移动状态
        focusTimer.start(100);

    }
    else
    {
        // 如果焦点器对象不存在，记录日志并发送错误消息
        Logger::Log("FocuserControlMoveStep | dpFocuser is NULL", LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient("FocusMoveDone:" + QString::number(0));
    }
    // 记录焦点器移动结束的日志
    Logger::Log("FocuserControlMoveStep finish!", LogLevel::INFO, DeviceType::FOCUSER);
}

void MainWindow::cancelStepMoveIfAny()
{
    // 清理可能残留的计时器与状态，避免重复连接/循环
    if (focusTimer.isActive()) focusTimer.stop();
    disconnect(&focusTimer, &QTimer::timeout, this, nullptr); // 断开所有连接
    isStepMoving = false;
}

int MainWindow::FocuserControl_setSpeed(int speed)
{
    Logger::Log("FocuserControl_setSpeed start ...", LogLevel::DEBUG, DeviceType::FOCUSER);
    if (dpFocuser != NULL)
    {
        int value, min, max, step;
        indi_Client->setFocuserSpeed(dpFocuser, speed);
        indi_Client->getFocuserSpeed(dpFocuser, value, min, max, step);
        Logger::Log("FocuserControl_setSpeed | Focuser Speed: " + std::to_string(value) + "," + std::to_string(min) + "," + std::to_string(max) + "," + std::to_string(step), LogLevel::INFO, DeviceType::FOCUSER);
        return value;
    }
    Logger::Log("FocuserControl_setSpeed finish!", LogLevel::DEBUG, DeviceType::FOCUSER);
}

int MainWindow::FocuserControl_getSpeed()
{
    Logger::Log("FocuserControl_getSpeed start ...", LogLevel::DEBUG, DeviceType::FOCUSER);
    if (dpFocuser != NULL)
    {
        int value, min, max, step;
        indi_Client->getFocuserSpeed(dpFocuser, value, min, max, step);
        Logger::Log("FocuserControl_getSpeed | Focuser Speed: " + std::to_string(value) + "," + std::to_string(min) + "," + std::to_string(max) + "," + std::to_string(step), LogLevel::INFO, DeviceType::FOCUSER);
        return value;
    }
    Logger::Log("FocuserControl_getSpeed finish!", LogLevel::DEBUG, DeviceType::FOCUSER);
}

int MainWindow::FocuserControl_getPosition()
{
    Logger::Log("FocuserControl_getPosition start ...", LogLevel::DEBUG, DeviceType::FOCUSER);
    if (dpFocuser != NULL)
    {
        int value;
        indi_Client->getFocuserAbsolutePosition(dpFocuser, value);
        Logger::Log("FocuserControl_getPosition | Focuser Position: " + std::to_string(value), LogLevel::DEBUG, DeviceType::FOCUSER);
        return value;
    }
    else
    {
        Logger::Log("FocuserControl_getPosition | dpFocuser is NULL", LogLevel::WARNING, DeviceType::FOCUSER);
        return 0; // 使用 INT_MIN 作为特殊的错误值
    }
    Logger::Log("FocuserControl_getPosition finish!", LogLevel::DEBUG, DeviceType::FOCUSER);
}

void MainWindow::TelescopeControl_Goto(double Ra, double Dec)
{
    if (dpMount != NULL)
    {
        if (indi_Client->mountState.isTracking)
        {
            indi_Client->slewTelescopeJNowNonBlock(dpMount, Ra, Dec, true);
        }
        else
        {
            indi_Client->slewTelescopeJNowNonBlock(dpMount, Ra, Dec, false);
        }
    }
}

QString MainWindow::TelescopeControl_Status()
{
    if (dpMount != NULL)
    {
        QString Stat;
        indi_Client->getTelescopeStatus(dpMount, Stat);
        return Stat;
    }
}

bool MainWindow::TelescopeControl_Park()
{
    bool isPark = false;
    if (dpMount != NULL)
    {
        indi_Client->getTelescopePark(dpMount, isPark);
        if (isPark == false)
        {
            indi_Client->setTelescopePark(dpMount, true);
        }
        else
        {
            indi_Client->setTelescopePark(dpMount, false);
        }
        indi_Client->getTelescopePark(dpMount, isPark);
        // Logger::Log("TelescopeControl_Park | Telescope is Park ???:" + std::to_string(isPark), LogLevel::INFO, DeviceType::MAIN);
    }

    return isPark;
}

bool MainWindow::TelescopeControl_Track()
{
    bool isTrack = true;
    if (dpMount != NULL)
    {
        indi_Client->getTelescopeTrackEnable(dpMount, isTrack);
        if (isTrack == false)
        {
            indi_Client->setTelescopeTrackEnable(dpMount, true);
        }
        else
        {
            indi_Client->setTelescopeTrackEnable(dpMount, false);
        }
        indi_Client->getTelescopeTrackEnable(dpMount, isTrack);
        Logger::Log("TelescopeControl_Track | Telescope is Track ???:" + std::to_string(isTrack), LogLevel::INFO, DeviceType::MAIN);
    }
    return isTrack;
}
void MainWindow::ScheduleTabelData(QString message)
{
    ScheduleTargetNames.clear();
    m_scheduList.clear();
    // 每次接收到新的任务计划表时，从第一条任务开始执行
    schedule_currentNum = 0;
    schedule_currentShootNum = 0;
    // 新任务计划表开始时，重置 Refocus 触发记录，避免旧任务残留导致本次不触发
    schedule_refocusTriggeredIndex = -1;
    QStringList ColDataList = message.split('[');
    for (int i = 1; i < ColDataList.size(); ++i)
    {
        QString ColData = ColDataList[i]; // ",M 24, Ra:4.785693,Dec:-0.323759,12:00:00,1 s,Ha,,Bias,ON,],"
        ScheduleData rowData;
        rowData.exposureDelay = 0; // 初始化曝光延迟为0
        qDebug() << "ColData[" << i << "]:" << ColData;

        QStringList RowDataList = ColData.split(',');
        if (RowDataList.size() <= 10)
        {
            Logger::Log(QString("ScheduleTabelData | row %1 has insufficient columns: %2").arg(i).arg(RowDataList.size()).toStdString(),
                        LogLevel::WARNING, DeviceType::MAIN);
            continue;
        }

        for (int j = 1; j <= 10; ++j)
        {
            // 防御性检查：确保索引有效
            if (j >= RowDataList.size())
            {
                Logger::Log(QString("ScheduleTabelData | row %1 column index %2 out of range (size=%3)")
                                .arg(i).arg(j).arg(RowDataList.size()).toStdString(),
                            LogLevel::WARNING, DeviceType::MAIN);
                break;
            }

            if (j == 1)
            {
                rowData.shootTarget = RowDataList[j];
                rowData.shootType = RowDataList[j + 7];
                qDebug() << "Target:" << rowData.shootTarget;
                qDebug() << "Type:" << rowData.shootType;
                // 将 shootTarget 添加到 ScheduleTargetNames 中
                if (!ScheduleTargetNames.isEmpty())
                {
                    ScheduleTargetNames += ",";
                }
                ScheduleTargetNames += rowData.shootTarget + "," + rowData.shootType;
            }
            else if (j == 2)
            {
                QStringList parts = RowDataList[j].split(':');
                if (parts.size() >= 2)
                {
                rowData.targetRa = Tools::RadToHour(parts[1].toDouble());
                }
                else
                {
                    rowData.targetRa = 0;
                    Logger::Log(QString("ScheduleTabelData | row %1 invalid RA field: %2").arg(i).arg(RowDataList[j]).toStdString(),
                                LogLevel::WARNING, DeviceType::MAIN);
                }
                qDebug() << "Ra:" << rowData.targetRa;
            }
            else if (j == 3)
            {
                QStringList parts = RowDataList[j].split(':');
                if (parts.size() >= 2)
                {
                rowData.targetDec = Tools::RadToDegree(parts[1].toDouble());
                }
                else
                {
                    rowData.targetDec = 0;
                    Logger::Log(QString("ScheduleTabelData | row %1 invalid Dec field: %2").arg(i).arg(RowDataList[j]).toStdString(),
                                LogLevel::WARNING, DeviceType::MAIN);
                }
                qDebug() << "Dec:" << rowData.targetDec;
            }
            else if (j == 4)
            {
                rowData.shootTime = RowDataList[j];
                qDebug() << "Time:" << rowData.shootTime;
            }
            else if (j == 5)
            {
                QStringList parts = RowDataList[j].split(' ');
                if (parts.isEmpty())
                {
                    rowData.exposureTime = 1000; // 默认 1s
                    Logger::Log(QString("ScheduleTabelData | row %1 invalid exposure field: %2, fallback to 1000ms")
                                    .arg(i).arg(RowDataList[j]).toStdString(),
                                LogLevel::WARNING, DeviceType::MAIN);
                    qDebug() << "Exptime error, Exptime = 1000 ms";
                    continue;
                }

                QString value = parts[0];
                QString unit = (parts.size() > 1) ? parts[1] : "s";
                if (unit == "s")
                    rowData.exposureTime = value.toInt() * 1000; // Convert seconds to milliseconds
                else if (unit == "ms")
                    rowData.exposureTime = value.toInt(); // Milliseconds
                if (rowData.exposureTime == 0)
                {
                    rowData.exposureTime = 1000;
                    qDebug() << "Exptime error, Exptime = 1000 ms";
                }
                qDebug() << "Exptime:" << rowData.exposureTime;
            }
            else if (j == 6)
            {
                rowData.filterNumber = RowDataList[j];
                qDebug() << "CFW:" << rowData.filterNumber;
            }
            else if (j == 7)
            {
                if (RowDataList[j] == "")
                {
                    rowData.repeatNumber = 1;
                    qDebug() << "Repeat error, Repeat = 1";
                }
                else
                {
                    rowData.repeatNumber = RowDataList[j].toInt();
                }
                qDebug() << "Repeat:" << rowData.repeatNumber;
            }
            // else if (j == 8)
            // {
            //     rowData.shootType = RowDataList[j];
            //     qDebug() << "Type:" << rowData.shootType;
            // }
            else if (j == 9)
            {
                rowData.resetFocusing = (RowDataList[j] == "ON");
                qDebug() << "Focus:" << rowData.resetFocusing;
            }
            else if (j == 10)
            {
                QStringList parts = RowDataList[j].split(' ');
                if (parts.isEmpty())
                {
                    rowData.exposureDelay = 0;
                    Logger::Log(QString("ScheduleTabelData | row %1 invalid exposure delay field: %2, fallback to 0")
                                    .arg(i).arg(RowDataList[j]).toStdString(),
                                LogLevel::WARNING, DeviceType::MAIN);
                    qDebug() << "Exposure Delay error, use 0 ms";
                    continue;
                }

                QString value = parts[0];
                QString unit = parts.size() > 1 ? parts[1] : "s";
                if (unit == "s")
                    rowData.exposureDelay = value.toInt() * 1000; // Convert seconds to milliseconds
                else if (unit == "ms")
                    rowData.exposureDelay = value.toInt(); // Milliseconds
                else
                    rowData.exposureDelay = 0; // Default to 0 if unit is not recognized
                qDebug() << "Exposure Delay:" << rowData.exposureDelay << "ms";
            }
        }
        rowData.progress = 0;
        // scheduleTable.Data.push_back(rowData);
        m_scheduList.append(rowData);
    }

    // 同步更新暂存的任务计划表数据，方便前端在刷新后通过 getStagingScheduleData 恢复当前计划
    // 前端发送格式为 "ScheduleTabelData:[,Target,...[,...]]"，这里复用内容，仅替换成 StagingScheduleData 前缀
    if (message.startsWith("ScheduleTabelData:"))
    {
        QString stagingMessage = message;
        stagingMessage.replace(0,
                               QString("ScheduleTabelData:").length(),
                               "StagingScheduleData:");
        isStagingScheduleData = true;
        StagingScheduleData = stagingMessage;
    }

    startSchedule();
}
void MainWindow::startSchedule()
{
    createScheduleDirectory();
    if (schedule_currentNum >= 0 && schedule_currentNum < m_scheduList.size())
    {
        qDebug() << "startSchedule...";
        schedule_ExpTime = m_scheduList[schedule_currentNum].exposureTime;
        schedule_RepeatNum = m_scheduList[schedule_currentNum].repeatNumber;
        schedule_CFWpos = m_scheduList[schedule_currentNum].filterNumber.toInt();
        schedule_ExposureDelay = m_scheduList[schedule_currentNum].exposureDelay;
        StopSchedule = false;
        isScheduleRunning = true;
        // 通知前端：计划任务当前处于运行状态
        emit wsThread->sendMessageToClient("ScheduleRunning:true");
        startTimeWaiting();
    }
    else
    {
        qDebug() << "Index out of range, Schedule is complete!";
        StopSchedule = true;
        isScheduleRunning = false;
        schedule_currentNum = 0;
        call_phd_StopLooping();
        GuidingHasStarted = false;
        // 通知前端计划任务已完成，重置按钮状态
        emit wsThread->sendMessageToClient("ScheduleComplete");
        emit wsThread->sendMessageToClient("ScheduleRunning:false");
        // 在实际应用中，你可能想要返回一个默认值或者处理索引超出范围的情况
        // 这里仅仅是一个简单的示例
        // return ScheduleData();
    }
}

void MainWindow::nextSchedule()
{
    // next schedule...
    schedule_currentNum++;
    qDebug() << "next schedule...";
    startSchedule();
}

void MainWindow::startTimeWaiting()
{
    // m_scheduList[schedule_currentNum].progress=0;
    // emit wsThread->sendMessageToClient("UpdateScheduleProcess:" + QString::number(schedule_currentNum) + ":" + QString::number(m_scheduList[schedule_currentNum].progress));
    qDebug() << "startTimeWaiting...";
    // 停止和清理先前的计时器
    timewaitingTimer.stop();
    timewaitingTimer.disconnect();
    // 启动等待的定时器
    timewaitingTimer.setSingleShot(true);
    connect(&timewaitingTimer, &QTimer::timeout, [this]()
            {
        if (StopSchedule)
        {
            StopSchedule = false;
            qDebug("Schedule is stop!");
            return;
        }

        if (WaitForTimeToComplete()) 
        {
            timewaitingTimer.stop();  // 完成时停止定时器
            qDebug() << "Time Waiting Complete...";
            m_scheduList[schedule_currentNum].progress = calculateScheduleProgress(1, 1.0);  // 步骤1完成：等待时间
            emit wsThread->sendMessageToClient("UpdateScheduleProcess:" + QString::number(schedule_currentNum) + ":" + QString::number(m_scheduList[schedule_currentNum].progress));
            emit wsThread->sendMessageToClient(
                "ScheduleStepState:" +
                QString::number(schedule_currentNum) + ":" +
                "wait:" +
                "0:" +
                "0:" +
                "100");

            startMountGoto(m_scheduList[schedule_currentNum].targetRa, m_scheduList[schedule_currentNum].targetDec);
        } 
        else 
        {
            timewaitingTimer.start(1000);  // 继续等待
        } });
    timewaitingTimer.start(1000);
}

void MainWindow::startMountGoto(double ra, double dec) // Ra:Hour, Dec:Degree
{
    if (dpMount == NULL)
    {
        m_scheduList[schedule_currentNum].progress = calculateScheduleProgress(2, 1.0);  // 无赤道仪时直接跳到步骤2完成
        emit wsThread->sendMessageToClient("UpdateScheduleProcess:" + QString::number(schedule_currentNum) + ":" + QString::number(m_scheduList[schedule_currentNum].progress));
        emit wsThread->sendMessageToClient(
            "ScheduleStepState:" +
            QString::number(schedule_currentNum) + ":" +
            "mount:" +
            "0:" +
            "0:" +
            "100");
        Logger::Log("startMountGoto | dpMount is NULL,goto failed!Skip to set CFW.", LogLevel::ERROR, DeviceType::MAIN);
        startSetCFW(schedule_CFWpos);
        return;
    }

    qDebug() << "startMountGoto...";
    // 停止和清理先前的计时器
    telescopeTimer.stop();
    telescopeTimer.disconnect();

    qDebug() << "Mount Goto:" << ra << "," << dec;
    MountGotoError = false;

    auto now = std::chrono::system_clock::now();
    double observatory_lon = 116.14; // 东经116.14度
    double lst = computeLST(observatorylongitude, now);

    // TelescopeControl_Goto(ra, dec);
    double RA_HOURS, DEC_DEGREE;
    indi_Client->getTelescopeRADECJNOW(dpMount, RA_HOURS, DEC_DEGREE);
    double CurrentRA_Degree = Tools::HourToDegree(RA_HOURS);
    double CurrentDEC_Degree = DEC_DEGREE;

    // performObservation(
    //     CurrentRA_Degree, CurrentDEC_Degree,
    //     ra, dec,
    //     observatorylongitude,observatorylatitude) ;
    // 在执行观测（GOTO）前，如当前处于导星状态，则暂时停止导星，待转动完成后再恢复
    pauseGuidingBeforeMountMove();

    performObservation(
        lst, CurrentDEC_Degree,
        ra, dec,
        observatorylongitude, observatorylatitude);

    sleep(2); // 赤道仪的状态更新有一定延迟

    // 步骤2：赤道仪转动，明确发送“开始移动”的细分信号，方便前端显示循环进度条
    // 这里将该步骤标记为进行中（本地进度 0.5），但 stepProgress 使用 0，表示刚开始。
    m_scheduList[schedule_currentNum].progress = calculateScheduleProgress(2, 0.5);
    emit wsThread->sendMessageToClient(
        "UpdateScheduleProcess:" +
        QString::number(schedule_currentNum) + ":" +
        QString::number(m_scheduList[schedule_currentNum].progress));
    emit wsThread->sendMessageToClient(
        "ScheduleStepState:" +
        QString::number(schedule_currentNum) + ":" +
        "mount:" +
        "0:" +
        "0:" +
        "0");

    // 启动等待赤道仪转动的定时器
    telescopeTimer.setSingleShot(true);

    connect(&telescopeTimer, &QTimer::timeout, [this]()
            {
        if (StopSchedule)
        {
            StopSchedule = false;
            qDebug("Schedule is stop!");

            if (dpMount != NULL)
            {
                indi_Client->setTelescopeAbortMotion(dpMount);
            }

            return;
        }
        if (WaitForTelescopeToComplete()) 
        {
            telescopeTimer.stop();  // 转动完成时停止定时器
            qDebug() << "Mount Goto Complete!";

            if(MountGotoError) {
                MountGotoError = false;

                nextSchedule();

                return;
            }

            // 如果本次 GOTO 之前处于导星状态，则在赤道仪转动完成后恢复导星
            resumeGuidingAfterMountMove();

            qDebug() << "Mount Goto Complete...";
            m_scheduList[schedule_currentNum].progress = calculateScheduleProgress(2, 1.0);  // 步骤2完成：赤道仪转动
            emit wsThread->sendMessageToClient("UpdateScheduleProcess:" + QString::number(schedule_currentNum) + ":" + QString::number(m_scheduList[schedule_currentNum].progress));
            emit wsThread->sendMessageToClient(
                "ScheduleStepState:" +
                QString::number(schedule_currentNum) + ":" +
                "mount:" +
                "0:" +
                "0:" +
                "100");
            startSetCFW(schedule_CFWpos);
        } 
        else 
        {
            telescopeTimer.start(1000);  // 继续等待
        } });

    telescopeTimer.start(1000);
}

bool MainWindow::needsMeridianFlip(double lst, double targetRA)
{
    double hourAngle = lst - targetRA;
    hourAngle = fmod(hourAngle + 24.0, 24.0);
    if (hourAngle > 12.0)
        hourAngle -= 24.0;
    return (hourAngle > 0);
}

void MainWindow::performObservation(
    double lst, double currentDec,
    double targetRA, double targetDec,
    double observatoryLongitude, double observatoryLatitude)
{
    // if (needsMeridianFlip(lst, targetRA))
    // {
    //     std::cout << "Meridian flip is needed." << std::endl;
    //     TelescopeControl_Goto(lst, observatoryLatitude);
    //     std::cout << "Performing meridian flip..." << std::endl;
    //     std::this_thread::sleep_for(std::chrono::seconds(60));
    //     TelescopeControl_Goto(targetRA, targetDec);
    // }
    // else
    // {
        std::cout << "No flip needed. Moving directly." << std::endl;
        TelescopeControl_Goto(targetRA, targetDec);
    // }
}

// 计算儒略日（JD）
double MainWindow::getJulianDate(const std::chrono::system_clock::time_point &utc_time)
{
    auto duration = utc_time.time_since_epoch();
    auto seconds = std::chrono::duration_cast<std::chrono::seconds>(duration).count();
    return 2440587.5 + seconds / 86400.0; // Unix时间转儒略日
}

// 计算格林尼治恒星时（GMST）
double MainWindow::computeGMST(const std::chrono::system_clock::time_point &utc_time)
{
    const double JD = getJulianDate(utc_time);
    const double T = (JD - 2451545.0) / 36525.0; // 以J2000为基准的世纪数

    // IAU公式（精度±0.1秒）
    double GMST = 280.46061837 + 360.98564736629 * (JD - 2451545.0) + 0.000387933 * T * T - T * T * T / 38710000.0;

    // 规范化到0-360度范围
    GMST = fmod(GMST, 360.0);
    if (GMST < 0)
        GMST += 360.0;

    return GMST / 15.0; // 转换为小时单位
}

// 计算地方恒星时（LST）
double MainWindow::computeLST(double longitude_east, const std::chrono::system_clock::time_point &utc_time)
{
    double GMST_hours = computeGMST(utc_time);
    double LST = GMST_hours + longitude_east / 15.0; // 东经为正
    return fmod(LST + 24.0, 24.0);                   // 确保结果在0-24之间
}

void MainWindow::startGuiding()
{
    qDebug() << "startGuiding...";
    // 停止和清理先前的计时器
    guiderTimer.stop();
    guiderTimer.disconnect();

    GuidingHasStarted = true;
    call_phd_StartLooping();
    sleep(2);
    call_phd_AutoFindStar();
    call_phd_StartGuiding();

    // 启动等待赤道仪转动的定时器
    guiderTimer.setSingleShot(true);

    connect(&guiderTimer, &QTimer::timeout, [this]()
            {
        if (StopSchedule)
        {
            StopSchedule = false;
            call_phd_StopLooping();
            qDebug("Schedule is stop!");
            return;
        }
        // 检查赤道仪状态
        if (WaitForGuidingToComplete()) 
        {
            guiderTimer.stop();  // 转动完成时停止定时器
            qDebug() << "Guiding Complete...";

            // startCapture(schedule_ExpTime);
            startSetCFW(schedule_CFWpos);
        } 
        else 
        {
            guiderTimer.start(1000);  // 继续等待
        } });

    guiderTimer.start(1000);
}

void MainWindow::startSetCFW(int pos)
{
    qDebug() << "startSetCFW...";
    
    // 检查是否需要自动对焦（Refocus为ON）
    if (schedule_currentNum >= 0 && schedule_currentNum < m_scheduList.size() && 
        m_scheduList[schedule_currentNum].resetFocusing &&
        schedule_refocusTriggeredIndex != schedule_currentNum)
    {
        qDebug() << "Refocus is ON, starting autofocus before setting CFW...";
        Logger::Log("计划任务表: Refocus为ON，在执行拍摄前先执行自动对焦（仅最后一步精调）", LogLevel::INFO, DeviceType::MAIN);

        // 先记录本行已触发过一次 Refocus，避免 AutoFocus 完成回调再次进入 startSetCFW 时无限重入
        schedule_refocusTriggeredIndex = schedule_currentNum;
        // 启动自动对焦（startScheduleAutoFocus会设置isScheduleTriggeredAutoFocus标志）
        startScheduleAutoFocus();
        return; // 自动对焦完成后会继续执行startSetCFW
    }
    
    // 如果不需要自动对焦，继续正常流程
    if (isFilterOnCamera)
    {
        if (dpMainCamera != NULL)
        {
            qDebug() << "schedule CFW pos:" << pos;
            m_scheduList[schedule_currentNum].progress = calculateScheduleProgress(3, 0.5);  // 步骤3进行中：开始设置滤镜
            emit wsThread->sendMessageToClient("UpdateScheduleProcess:" + QString::number(schedule_currentNum) + ":" + QString::number(m_scheduList[schedule_currentNum].progress));
            emit wsThread->sendMessageToClient(
                "ScheduleStepState:" +
                QString::number(schedule_currentNum) + ":" +
                "filter:" +
                "0:" +
                "0:" +
                "50");
            indi_Client->setCFWPosition(dpMainCamera, pos);
            qDebug() << "CFW Goto Complete...";
            startCapture(schedule_ExpTime);
            m_scheduList[schedule_currentNum].progress = calculateScheduleProgress(3, 1.0);  // 步骤3完成：滤镜设置完成
            emit wsThread->sendMessageToClient("UpdateScheduleProcess:" + QString::number(schedule_currentNum) + ":" + QString::number(m_scheduList[schedule_currentNum].progress));
            emit wsThread->sendMessageToClient(
                "ScheduleStepState:" +
                QString::number(schedule_currentNum) + ":" +
                "filter:" +
                "0:" +
                "0:" +
                "100");
        }
        else
        {
            Logger::Log("startSetCFW | dpMainCamera is NULL,set CFW failed!", LogLevel::ERROR, DeviceType::MAIN);

            m_scheduList[schedule_currentNum].progress = calculateScheduleProgress(3, 1.0);  // 无滤镜时直接跳到步骤3完成
            emit wsThread->sendMessageToClient("UpdateScheduleProcess:" + QString::number(schedule_currentNum) + ":" + QString::number(m_scheduList[schedule_currentNum].progress));
            startCapture(schedule_ExpTime);
        }
    }
    else
    {
        if (dpCFW != NULL)
        {
            qDebug() << "schedule CFW pos:" << pos;
            m_scheduList[schedule_currentNum].progress = calculateScheduleProgress(3, 0.5);  // 步骤3进行中：开始设置滤镜
            emit wsThread->sendMessageToClient("UpdateScheduleProcess:" + QString::number(schedule_currentNum) + ":" + QString::number(m_scheduList[schedule_currentNum].progress));
            emit wsThread->sendMessageToClient(
                "ScheduleStepState:" +
                QString::number(schedule_currentNum) + ":" +
                "filter:" +
                "0:" +
                "0:" +
                "50");
            indi_Client->setCFWPosition(dpCFW, pos);
            qDebug() << "CFW Goto Complete...";
            m_scheduList[schedule_currentNum].progress = calculateScheduleProgress(3, 1.0);  // 步骤3完成：滤镜设置完成
            emit wsThread->sendMessageToClient("UpdateScheduleProcess:" + QString::number(schedule_currentNum) + ":" + QString::number(m_scheduList[schedule_currentNum].progress));
            emit wsThread->sendMessageToClient(
                "ScheduleStepState:" +
                QString::number(schedule_currentNum) + ":" +
                "filter:" +
                "0:" +
                "0:" +
                "100");
            startCapture(schedule_ExpTime);
        }
        else
        {
            Logger::Log("startSetCFW | dpCFW is NULL,set CFW failed!", LogLevel::ERROR, DeviceType::MAIN);
            m_scheduList[schedule_currentNum].progress = calculateScheduleProgress(3, 1.0);  // 无滤镜时直接跳到步骤3完成
            emit wsThread->sendMessageToClient("UpdateScheduleProcess:" + QString::number(schedule_currentNum) + ":" + QString::number(m_scheduList[schedule_currentNum].progress));
            startCapture(schedule_ExpTime);
        }
    }
}

void MainWindow::startExposureDelay()
{
    qDebug() << "startExposureDelay...";
    Logger::Log(("Waiting exposure delay: " + QString::number(schedule_ExposureDelay) + " ms before next capture").toStdString(), LogLevel::INFO, DeviceType::MAIN);
    qDebug() << "Waiting exposure delay:" << schedule_ExposureDelay << "ms before next capture";
    
    // 停止和清理先前的延迟定时器
    exposureDelayTimer.stop();
    exposureDelayTimer.disconnect();
    
    // 重置已过去的时间
    exposureDelayElapsed_ms = 0;
    
    // 使用可控制的定时器，每100ms检查一次
    exposureDelayTimer.setSingleShot(false);
    // 向前端发送步骤状态：进入延时阶段
    emit wsThread->sendMessageToClient(
        "ScheduleStepState:" +
        QString::number(schedule_currentNum) + ":" +
        "delay:" +
        "0:" +
        "0:" +
        "0");
    connect(&exposureDelayTimer, &QTimer::timeout, [this]() {
        // 首先检查是否已停止，必须在最开始检查
        // 这个检查必须在任何其他操作之前，确保能够立即响应停止信号
        if (StopSchedule)
        {
            // 立即停止定时器并清理
            exposureDelayTimer.stop();
            exposureDelayTimer.disconnect();
            Logger::Log(("Exposure delay interrupted: Schedule stopped during delay wait (elapsed: " + QString::number(exposureDelayElapsed_ms) + " ms)").toStdString(), LogLevel::INFO, DeviceType::MAIN);
            qDebug() << "Exposure delay interrupted: Schedule stopped during delay wait (elapsed:" << exposureDelayElapsed_ms << "ms)";
            exposureDelayElapsed_ms = 0; // 重置已过去的时间
            return; // 立即返回，不执行任何后续操作
        }
        
        // 检查定时器是否仍然激活（防止在回调执行期间被停止）
        if (!exposureDelayTimer.isActive())
        {
            qDebug() << "Exposure delay timer is not active, returning";
            return;
        }
        
        // 增加已过去的时间
        exposureDelayElapsed_ms += 100; // 每次增加100ms
        
        // 再次检查 StopSchedule（可能在增加时间的过程中被设置）
        if (StopSchedule)
        {
            exposureDelayTimer.stop();
            exposureDelayTimer.disconnect();
            Logger::Log(("Exposure delay interrupted: Schedule stopped during delay wait (elapsed: " + QString::number(exposureDelayElapsed_ms) + " ms)").toStdString(), LogLevel::INFO, DeviceType::MAIN);
            qDebug() << "Exposure delay interrupted: Schedule stopped during delay wait (elapsed:" << exposureDelayElapsed_ms << "ms)";
            exposureDelayElapsed_ms = 0; // 重置已过去的时间
            return; // 立即返回，不执行任何后续操作
        }
        
        // 向前端报告当前延迟阶段的进度（0-100）
        if (schedule_ExposureDelay > 0)
        {
            int progress = static_cast<int>(
                qMin(100.0, exposureDelayElapsed_ms * 100.0 / static_cast<double>(schedule_ExposureDelay)));
            emit wsThread->sendMessageToClient(
                "ScheduleStepState:" +
                QString::number(schedule_currentNum) + ":" +
                "delay:" +
                "0:" +
                "0:" +
                QString::number(progress));
        }
        
        // 检查是否已经过了延迟时间
        if (exposureDelayElapsed_ms >= schedule_ExposureDelay)
        {
            exposureDelayTimer.stop();
            exposureDelayTimer.disconnect();
            qDebug() << "Exposure delay complete, starting next capture";
            exposureDelayElapsed_ms = 0; // 重置已过去的时间
            emit wsThread->sendMessageToClient(
                "ScheduleStepState:" +
                QString::number(schedule_currentNum) + ":" +
                "delay:" +
                "0:" +
                "0:" +
                "100");
            startCapture(schedule_ExpTime);
        }
    });
    
    exposureDelayTimer.start(100); // 每100ms检查一次
}

void MainWindow::startCapture(int ExpTime)
{
    qDebug() << "startCapture...";
    // 停止和清理先前的计时器
    captureTimer.stop();
    captureTimer.disconnect();
    // 停止曝光延迟定时器（如果正在运行）
    exposureDelayTimer.stop();
    exposureDelayTimer.disconnect();

    ShootStatus = "InProgress";
    qDebug() << "ShootStatus: " << ShootStatus;
    INDI_Capture(ExpTime);
    schedule_currentShootNum++;

    captureTimer.setSingleShot(true);

    expTime_ms = 0;

    connect(&captureTimer, &QTimer::timeout, [this]()
            {
        if (StopSchedule)
        {
            StopSchedule = false;
            INDI_AbortCapture();
            qDebug("Schedule is stop!");
            return;
        }
        
        // 检查赤道仪状态
        if (WaitForShootToComplete()) 
        {
            captureTimer.stop();  // 转动完成时停止定时器
            qDebug() << "Capture" << schedule_currentShootNum << "Complete!";
            ScheduleImageSave(m_scheduList[schedule_currentNum].shootTarget, schedule_currentShootNum);
            
            // 计算当前拍摄完成的进度
            // 拍摄步骤从步骤4开始，每张照片对应一个步骤
            // 步骤编号 = 3 + schedule_currentShootNum
            int currentStep = 3 + schedule_currentShootNum;
            m_scheduList[schedule_currentNum].progress = calculateScheduleProgress(currentStep, 1.0);
            emit wsThread->sendMessageToClient("UpdateScheduleProcess:" + QString::number(schedule_currentNum) + ":" + QString::number(m_scheduList[schedule_currentNum].progress));
            emit wsThread->sendMessageToClient(
                "ScheduleStepState:" +
                QString::number(schedule_currentNum) + ":" +
                "exposure:" +
                QString::number(schedule_currentShootNum) + ":" +
                QString::number(schedule_RepeatNum) + ":" +
                "100");

            // 同步更新循环进度（loopDone）：每完成一张就回传当前已完成张数和总张数，
            // 便于前端实时显示 “已拍/总拍” 的精细进度，而不是只在最后一次拍摄结束时一次性更新。
            if (schedule_RepeatNum > 0)
            {
                int loopProgress = static_cast<int>(
                    qMin(100.0, schedule_currentShootNum * 100.0 / static_cast<double>(schedule_RepeatNum)));
                emit wsThread->sendMessageToClient(
                    // 专用循环状态信号：ScheduleLoopState:row:loopDone:loopTotal:progress
                    "ScheduleLoopState:" +
                    QString::number(schedule_currentNum) + ":" +
                    QString::number(schedule_currentShootNum) + ":" +
                    QString::number(schedule_RepeatNum) + ":" +
                    QString::number(loopProgress));
            }

            if (schedule_currentShootNum < schedule_RepeatNum)
            {
                // 如果有曝光延迟，等待指定时间后再开始下一张
                if (schedule_ExposureDelay > 0)
                {
                    startExposureDelay();
                }
                else
                {
                    // 没有延迟，直接开始下一张
                    startCapture(schedule_ExpTime);
                }
            }
            else
            {
                schedule_currentShootNum = 0;

                m_scheduList[schedule_currentNum].progress=100;
                emit wsThread->sendMessageToClient("UpdateScheduleProcess:" + QString::number(schedule_currentNum) + ":" + QString::number(m_scheduList[schedule_currentNum].progress));   
                qDebug() << "Capture Goto Complete...";
                nextSchedule();
            }

        } 
        else 
        {
            expTime_ms += 1000;
            
            // 检查是否超过最大超时时间（曝光时间 + 1分钟）
            int maxTimeout = schedule_ExpTime + 60000;  // 曝光时间 + 60000毫秒（1分钟）
            if (expTime_ms > maxTimeout)
            {
                // 拍摄超时，中止拍摄并处理超时情况
                captureTimer.stop();
                INDI_AbortCapture();
                Logger::Log(QString("计划任务表拍摄超时: 当前拍摄时间 %1ms, 超过最大超时时间 %2ms (曝光时间 %3ms + 1分钟)").arg(expTime_ms).arg(maxTimeout).arg(schedule_ExpTime).toStdString(), 
                           LogLevel::WARNING, DeviceType::MAIN);
                qDebug() << "Capture timeout! expTime_ms:" << expTime_ms << ", maxTimeout:" << maxTimeout << ", schedule_ExpTime:" << schedule_ExpTime;
                
                // 跳过当前拍摄，继续下一个拍摄或任务
                if (schedule_currentShootNum < schedule_RepeatNum)
                {
                    // 还有后续拍摄，继续下一个拍摄
                    qDebug() << "Skip current capture, continue to next capture...";
                    // 如果有曝光延迟，等待指定时间后再开始下一张
                    if (schedule_ExposureDelay > 0)
                    {
                        startExposureDelay();
                    }
                    else
                    {
                        // 没有延迟，直接开始下一张
                        startCapture(schedule_ExpTime);
                    }
                }
                else
                {
                    // 当前任务的所有拍摄已完成或超时，进入下一个任务
                    schedule_currentShootNum = 0;
                    qDebug() << "All captures completed or timeout, move to next schedule...";
                    m_scheduList[schedule_currentNum].progress = 100;
                    emit wsThread->sendMessageToClient("UpdateScheduleProcess:" + QString::number(schedule_currentNum) + ":" + QString::number(m_scheduList[schedule_currentNum].progress));
                    emit wsThread->sendMessageToClient(
                        // 专用循环状态信号收尾：全部完成时回传 100%
                        "ScheduleLoopState:" +
                        QString::number(schedule_currentNum) + ":" +
                        QString::number(schedule_RepeatNum) + ":" +
                        QString::number(schedule_RepeatNum) + ":" +
                        "100");
                    nextSchedule();
                }
                return;
            }
            
            // 计算拍摄过程中的进度
            // 拍摄步骤从步骤4开始，每张照片对应一个步骤
            // 步骤编号 = 3 + schedule_currentShootNum
            // stepProgress = 当前曝光进度（0.0-1.0）
            int currentStep = 3 + schedule_currentShootNum;
            double shotProgress = qMin(expTime_ms / (double)schedule_ExpTime, 1.0);  // 限制在0.0-1.0之间

            // 为避免曝光时间较长时频繁刷新“当前总进度”导致前端整体进度条跳动混乱，
            // 这里不再在曝光进行过程中更新 m_scheduList[schedule_currentNum].progress，
            // 仅通过 ScheduleStepState 将当前曝光的细粒度进度（0-100%）回传给前端用于单步倒计时显示。
            qDebug() << "expTime_ms:" << expTime_ms << ", schedule_ExpTime:" << schedule_ExpTime 
                     << ", currentShootNum:" << schedule_currentShootNum << ", RepeatNum:" << schedule_RepeatNum
                     << ", currentStep:" << currentStep << ", shotProgress:" << shotProgress;

            emit wsThread->sendMessageToClient(
                "ScheduleStepState:" +
                QString::number(schedule_currentNum) + ":" +
                "exposure:" +
                QString::number(schedule_currentShootNum) + ":" +
                QString::number(schedule_RepeatNum) + ":" +
                QString::number(static_cast<int>(shotProgress * 100.0)));
            captureTimer.start(1000);  // 继续等待
        } });

    captureTimer.start(1000);
}

bool MainWindow::WaitForTelescopeToComplete()
{
    return (TelescopeControl_Status() != "Moving");
}

bool MainWindow::WaitForShootToComplete()
{
    qInfo("Wait For Shoot To Complete...");
    return (ShootStatus != "InProgress");
}

bool MainWindow::WaitForGuidingToComplete()
{
    qInfo() << "Wait For Guiding To Complete..." << InGuiding;
    return InGuiding;
}

bool MainWindow::WaitForTimeToComplete()
{
    qInfo() << "Wait For Time To Complete...";
    QString TargetTime = m_scheduList[schedule_currentNum].shootTime;

    // 如果获取到的目标时间不是完整的时间格式，直接返回 true
    if (TargetTime.length() != 5 || TargetTime[2] != ':')
        return true;

    // 获取当前时间
    QTime currentTime = QTime::currentTime();
    // 解析目标时间
    QTime targetTime = QTime::fromString(TargetTime, "hh:mm");

    qInfo() << "currentTime:" << currentTime << ", targetTime:" << targetTime;

    // 如果目标时间晚于当前时间，返回 false
    if (targetTime > currentTime)
        return false;

    // 目标时间早于或等于当前时间，返回 true
    return true;
}

int MainWindow::calculateScheduleProgress(int stepNumber, double stepProgress)
{
    // 步骤定义：
    // 步骤1：等待时间
    // 步骤2：赤道仪转动
    // 步骤3：滤镜设置
    // 步骤4到4+RepeatNum-1：拍摄（每张照片算一个步骤）
    // 总步骤数 = 3 + schedule_RepeatNum
    
    int totalSteps = 3 + schedule_RepeatNum;
    if (totalSteps <= 0)
    {
        return 100;  // 如果总步骤数为0，直接返回100%
    }
    
    // 计算每步的进度增量
    double progressPerStep = 100.0 / totalSteps;
    
    // 计算当前步骤的进度
    // stepProgress 用于步骤内的进度（0.0-1.0），例如拍摄过程中的进度
    double currentProgress = stepNumber * progressPerStep * stepProgress;
    
    // 如果超过100%，强制转换为100%
    if (currentProgress > 100.0)
    {
        currentProgress = 100.0;
    }
    
    return static_cast<int>(currentProgress);
}


int MainWindow::CaptureImageSave()
{
    qDebug() << "CaptureImageSave...";
    const char *sourcePath = "/dev/shm/ccd_simulator.fits";

    if (!QFile::exists("/dev/shm/ccd_simulator.fits"))
    {
        emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Null");
        return 1;
    }

    QString CaptureTime = Tools::getFitsCaptureTime("/dev/shm/ccd_simulator.fits");
    QRegularExpression nonAlphaNumericRx("[^a-zA-Z0-9]");
    QString tempCaptureTime = CaptureTime;
    QRegularExpressionMatchIterator i = nonAlphaNumericRx.globalMatch(CaptureTime);
    while (i.hasNext()) {
        QRegularExpressionMatch match = i.next();
        tempCaptureTime.replace(match.capturedStart(), match.capturedLength(), "_");
    }
    CaptureTime = tempCaptureTime;
    QString resultFileName = CaptureTime + ".fits";

    std::time_t currentTime = std::time(nullptr);
    std::tm *timeInfo = std::localtime(&currentTime);
    char buffer[80];
    std::strftime(buffer, 80, "%Y-%m-%d", timeInfo); // Format: YYYY-MM-DD

    // 直接使用 ImageSaveBaseDirectory（无论是默认路径还是U盘路径）
    QString destinationDirectory = ImageSaveBaseDirectory + "/CaptureImage";
    QString destinationPath = destinationDirectory + "/" + QString(buffer) + "/" + resultFileName;
    
    // 判断是否为U盘路径（使用saveMode参数）
    bool isUSBSave = (saveMode != "local");
    
    // 使用通用函数检查存储空间并创建目录
    QString dirPathToCreate = isUSBSave ? (destinationDirectory + "/" + QString(buffer)) : QString();
    int checkResult = checkStorageSpaceAndCreateDirectory(
        sourcePath,
        destinationDirectory,
        dirPathToCreate,
        "CaptureImageSave",
        isUSBSave,
        [this]() { createCaptureDirectory(); }
    );
    if (checkResult != 0)
    {
        return checkResult;
    }

    // 检查文件是否已存在
    if (QFile::exists(destinationPath))
    {
        qWarning() << "The file already exists, there is no need to save it again:" << destinationPath;
        emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Repeat");
        return 0;
    }

    // 使用通用函数保存文件
    int saveResult = saveImageFile(sourcePath, destinationPath, "CaptureImageSave", isUSBSave);
    if (saveResult != 0)
    {
        return saveResult;
    }

    emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Success");
    Logger::Log("CaptureImageSave | File saved successfully: " + destinationPath.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    return 0;
}

int MainWindow::ScheduleImageSave(QString name, int num)
{
    qDebug() << "ScheduleImageSave...";
    const char *sourcePath = "/dev/shm/ccd_simulator.fits";

    name.replace(' ', '_');
    QString resultFileName = QString("%1-%2.fits").arg(name).arg(num);

    std::time_t currentTime = std::time(nullptr);
    std::tm *timeInfo = std::localtime(&currentTime);
    char buffer[80];
    std::strftime(buffer, 80, "%Y-%m-%d", timeInfo); // Format: YYYY-MM-DD

    // 指定目标目录
    QString destinationDirectory = ImageSaveBaseDirectory + "/ScheduleImage";

    // 拼接目标文件路径
    QString destinationPath = destinationDirectory + "/" + buffer + " " + QTime::currentTime().toString("hh") + "h (" + ScheduleTargetNames + ")" + "/" + resultFileName;
    
    // 判断是否为U盘路径（使用saveMode参数）
    bool isUSBSave = (saveMode != "local");
    
    // 使用通用函数检查存储空间并创建目录
    QString dirPathToCreate = isUSBSave ? (destinationDirectory + "/" + QString(buffer) + " " + QTime::currentTime().toString("hh") + "h (" + ScheduleTargetNames + ")") : QString();
    int checkResult = checkStorageSpaceAndCreateDirectory(
        sourcePath,
        destinationDirectory,
        dirPathToCreate,
        "ScheduleImageSave",
        isUSBSave,
        [this]() { createScheduleDirectory(); }
    );
    if (checkResult != 0)
    {
        return checkResult;
    }

    // 使用通用函数保存文件
    int saveResult = saveImageFile(sourcePath, destinationPath, "ScheduleImageSave", isUSBSave);
    if (saveResult != 0)
    {
        return saveResult;
    }
    
    qDebug() << "ScheduleImageSave Goto Complete...";
    emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Success");
    return 0;
}
int MainWindow::solveFailedImageSave(const QString& imagePath)
{
    qDebug() << "solveFailedImageSave...";
    Logger::Log("solveFailedImageSave | Starting save process, imagePath: " + imagePath.toStdString(), LogLevel::INFO, DeviceType::MAIN);

    // 如果未提供路径，使用默认路径
    QString sourcePathStr = imagePath.isEmpty() ? "/dev/shm/ccd_simulator.fits" : imagePath;
    const char *sourcePath = sourcePathStr.toLocal8Bit().constData();

    Logger::Log("solveFailedImageSave | Using source path: " + sourcePathStr.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    
    if (!QFile::exists(sourcePathStr))
    {
        Logger::Log("solveFailedImageSave | 文件不存在: " + sourcePathStr.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Null");
        return 1;
    }
    
    Logger::Log("solveFailedImageSave | Source file exists, file size: " + std::to_string(QFileInfo(sourcePathStr).size()) + " bytes", LogLevel::INFO, DeviceType::MAIN);

    QString CaptureTime = Tools::getFitsCaptureTime(sourcePath);
    Logger::Log("solveFailedImageSave | getFitsCaptureTime returned: " + CaptureTime.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    
    // 如果无法从 FITS 文件获取时间，使用当前时间戳
    if (CaptureTime.isEmpty())
    {
        std::time_t currentTime = std::time(nullptr);
        std::tm *timeInfo = std::localtime(&currentTime);
        char buffer[80];
        std::strftime(buffer, 80, "%Y_%m_%dT%H_%M_%S", timeInfo);
        CaptureTime = QString::fromStdString(buffer);
        Logger::Log("solveFailedImageSave | Using current timestamp as filename: " + CaptureTime.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    }
    
    QRegularExpression nonAlphaNumericRx("[^a-zA-Z0-9]");
    QString tempCaptureTime = CaptureTime;
    QRegularExpressionMatchIterator i = nonAlphaNumericRx.globalMatch(CaptureTime);
    while (i.hasNext()) {
        QRegularExpressionMatch match = i.next();
        tempCaptureTime.replace(match.capturedStart(), match.capturedLength(), "_");
    }
    CaptureTime = tempCaptureTime;
    QString resultFileName = CaptureTime + ".fits";
    Logger::Log("solveFailedImageSave | Generated filename: " + resultFileName.toStdString(), LogLevel::INFO, DeviceType::MAIN);

    std::time_t currentTime = std::time(nullptr);
    std::tm *timeInfo = std::localtime(&currentTime);
    char buffer[80];
    std::strftime(buffer, 80, "%Y-%m-%d", timeInfo); // Format: YYYY-MM-DD

    // 指定目标目录
    QString destinationDirectory = ImageSaveBaseDirectory + "/solveFailedImage";

    QString destinationPath = destinationDirectory + "/" + buffer + "/" + resultFileName;
    
    // 判断是否为U盘路径（使用saveMode参数）
    bool isUSBSave = (saveMode != "local");
    
    // 使用通用函数检查存储空间并创建目录
    // 注意：传入 QString 而不是 const char*，确保路径正确传递
    QString dirPathToCreate = isUSBSave ? (destinationDirectory + "/" + QString(buffer)) : QString();
    
    // 在调用前再次确认文件存在（因为文件可能在检查后被删除）
    if (!QFile::exists(sourcePathStr))
    {
        Logger::Log("solveFailedImageSave | Source file no longer exists before checkStorageSpaceAndCreateDirectory: " + sourcePathStr.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Null");
        return 1;
    }
    
    int checkResult = checkStorageSpaceAndCreateDirectory(
        sourcePathStr,  // 使用 QString 而不是 const char*
        destinationDirectory,
        dirPathToCreate,
        "solveFailedImageSave",
        isUSBSave,
        [this]() { createsolveFailedImageDirectory(); }
    );
    if (checkResult != 0)
    {
        Logger::Log("solveFailedImageSave | checkStorageSpaceAndCreateDirectory failed with code: " + std::to_string(checkResult), LogLevel::ERROR, DeviceType::MAIN);
        return checkResult;
    }

    // 检查文件是否已存在
    // if (QFile::exists(destinationPath))
    // {
    //     qWarning() << "The file already exists, there is no need to save it again:" << destinationPath;
    //     emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Repeat");
    //     return 0;
    // }

    // 使用通用函数保存文件
    // 在保存前再次确认源文件存在
    if (!QFile::exists(sourcePathStr))
    {
        Logger::Log("solveFailedImageSave | Source file no longer exists before saveImageFile: " + sourcePathStr.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Null");
        return 1;
    }
    
    Logger::Log("solveFailedImageSave | Attempting to save to: " + destinationPath.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    int saveResult = saveImageFile(sourcePathStr, destinationPath, "solveFailedImageSave", isUSBSave);  // 使用 QString 而不是 const char*
    if (saveResult != 0)
    {
        Logger::Log("solveFailedImageSave | saveImageFile failed with error code: " + std::to_string(saveResult), LogLevel::ERROR, DeviceType::MAIN);
        return saveResult;
    }

    // 验证文件是否真的被保存了
    if (QFile::exists(destinationPath))
    {
        Logger::Log("solveFailedImageSave | File saved successfully to: " + destinationPath.toStdString() + ", size: " + std::to_string(QFileInfo(destinationPath).size()) + " bytes", LogLevel::INFO, DeviceType::MAIN);
    }
    else
    {
        Logger::Log("solveFailedImageSave | WARNING: saveImageFile returned success but destination file does not exist: " + destinationPath.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
    }

    emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Success");
    qDebug() << "CaptureImageSaveStatus Goto Complete...";
    return 0;
}

bool MainWindow::directoryExists(const std::string &path)
{
    struct stat info;
    return stat(path.c_str(), &info) == 0 && S_ISDIR(info.st_mode);
}

bool MainWindow::createScheduleDirectory()
{
    Logger::Log("createScheduleDirectory start ...", LogLevel::INFO, DeviceType::MAIN);
    std::string basePath = ImageSaveBasePath + "/ScheduleImage";

    std::time_t currentTime = std::time(nullptr);
    std::tm *timeInfo = std::localtime(&currentTime);
    char buffer[80];
    std::strftime(buffer, 80, "%Y-%m-%d", timeInfo); // Format: YYYY-MM-DD
    std::string folderName = basePath + "/" + buffer + " " + QTime::currentTime().toString("hh").toStdString() + "h (" + ScheduleTargetNames.toStdString() + ")";

    // 如果目录不存在，则创建（使用 create_directories 创建多层目录）
    if (!std::filesystem::exists(folderName))
    {
        if (std::filesystem::create_directories(folderName))
        {
            Logger::Log("createScheduleDirectory | Folder created successfully: " + std::string(folderName), LogLevel::INFO, DeviceType::MAIN);
        }
        else
        {
            Logger::Log("createScheduleDirectory | An error occurred while creating the folder.", LogLevel::ERROR, DeviceType::MAIN);
            return false;
        }
    }
    else
    {
        Logger::Log("createScheduleDirectory | The folder already exists: " + std::string(folderName), LogLevel::INFO, DeviceType::MAIN);
    }
    return true;
}
bool MainWindow::createCaptureDirectory()
{
    Logger::Log("createCaptureDirectory start ...", LogLevel::INFO, DeviceType::MAIN);
    std::string basePath = ImageSaveBasePath + "/CaptureImage/";

    std::time_t currentTime = std::time(nullptr);
    std::tm *timeInfo = std::localtime(&currentTime);
    char buffer[80];
    std::strftime(buffer, 80, "%Y-%m-%d", timeInfo); // Format: YYYY-MM-DD
    std::string folderName = basePath + buffer;

    // 如果目录不存在，则创建
    if (!std::filesystem::exists(folderName))
    {
        if (std::filesystem::create_directory(folderName))
        {
            Logger::Log("createCaptureDirectory | Folder created successfully: " + std::string(folderName), LogLevel::INFO, DeviceType::MAIN);
        }
        else
        {
            Logger::Log("createCaptureDirectory | An error occurred while creating the folder.", LogLevel::INFO, DeviceType::MAIN);
        }
    }
    else
    {
        Logger::Log("createCaptureDirectory | The folder already exists: " + std::string(folderName), LogLevel::INFO, DeviceType::MAIN);
    }
}
bool MainWindow::createsolveFailedImageDirectory()
{
    Logger::Log("createCaptureDirectory start ...", LogLevel::INFO, DeviceType::MAIN);
    std::string basePath = ImageSaveBasePath + "/solveFailedImage/";

    std::time_t currentTime = std::time(nullptr);
    std::tm *timeInfo = std::localtime(&currentTime);
    char buffer[80];
    std::strftime(buffer, 80, "%Y-%m-%d", timeInfo); // Format: YYYY-MM-DD
    std::string folderName = basePath + buffer;

    // 如果目录不存在，则创建
    if (!std::filesystem::exists(folderName))
    {
        if (std::filesystem::create_directory(folderName))
        {
            Logger::Log("createCaptureDirectory | Folder created successfully: " + std::string(folderName), LogLevel::INFO, DeviceType::MAIN);
        }
        else
        {
            Logger::Log("createCaptureDirectory | An error occurred while creating the folder.", LogLevel::INFO, DeviceType::MAIN);
        }
    }
    else
    {
        Logger::Log("createCaptureDirectory | The folder already exists: " + std::string(folderName), LogLevel::INFO, DeviceType::MAIN);
    }
}

int MainWindow::checkStorageSpaceAndCreateDirectory(const QString &sourcePath, 
                                                     const QString &destinationDirectory,
                                                     const QString &dirPathToCreate,
                                                     const QString &functionName,
                                                     bool isUSBSave,
                                                     std::function<void()> createLocalDirectoryFunc)
{
    Logger::Log(functionName.toStdString() + " | checkStorageSpaceAndCreateDirectory | saveMode: " + saveMode.toStdString() + 
               ", isUSBSave: " + std::string(isUSBSave ? "true" : "false") + 
               ", ImageSaveBaseDirectory: " + ImageSaveBaseDirectory.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    
    // 先获取源文件大小（在空间检查之前）
    QFileInfo sourceFileInfo(sourcePath);
    if (!sourceFileInfo.exists())
    {
        Logger::Log(functionName.toStdString() + " | Source file does not exist.", LogLevel::WARNING, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
        return 1;
    }
    long long fileSize = sourceFileInfo.size();
    
    if (isUSBSave)
    {
        // 从ImageSaveBaseDirectory提取U盘挂载点（去掉/QUARCS_ImageSave）
        QString usb_mount_point = ImageSaveBaseDirectory;
        usb_mount_point.replace("/QUARCS_ImageSave", "");
        
        Logger::Log(functionName.toStdString() + " | USB save mode | ImageSaveBaseDirectory: " + ImageSaveBaseDirectory.toStdString() + 
                   ", extracted USB mount point: " + usb_mount_point.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        
        // 检查U盘空间和可写性
        QStorageInfo storageInfo(usb_mount_point);
        if (!storageInfo.isValid() || !storageInfo.isReady())
        {
            Logger::Log(functionName.toStdString() + " | USB drive is not valid or not ready: " + usb_mount_point.toStdString() + 
                       " (isValid: " + std::string(storageInfo.isValid() ? "true" : "false") + 
                       ", isReady: " + std::string(storageInfo.isReady() ? "true" : "false") + ")", LogLevel::WARNING, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("CaptureImageSaveStatus:USB-NotAvailable");
            return 1;
        }
        
        if (storageInfo.isReadOnly())
        {
            //const QString password = "quarcs";
            //if (!remountReadWrite(usb_mount_point, password))
            //{
                Logger::Log(functionName.toStdString() + " | Failed to remount USB as read-write.", LogLevel::WARNING, DeviceType::MAIN);
                emit wsThread->sendMessageToClient("CaptureImageSaveStatus:USB-ReadOnly");
            //   return 1;
            //}
        }
        
        // 检查U盘剩余空间（在创建目录之前）
        long long remaining_space = getUSBSpace(usb_mount_point);
        if (remaining_space == -1 || remaining_space <= 0)
        {
            Logger::Log(functionName.toStdString() + " | USB drive has no available space.", LogLevel::WARNING, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("CaptureImageSaveStatus:USB-NoSpace");
            return 1;
        }
        
        // 预留至少100MB的缓冲空间，避免写入时空间不足
        const long long RESERVE_SPACE = 100 * 1024 * 1024; // 100MB
        long long available_space = remaining_space - RESERVE_SPACE;
        if (available_space < 0)
        {
            available_space = 0;
        }
        
        // 检查空间是否足够（文件大小必须小于可用空间，已预留缓冲）
        if (fileSize > available_space)
        {
            Logger::Log(functionName.toStdString() + " | Insufficient USB space. Required: " + QString::number(fileSize).toStdString() + 
                       " bytes, Available: " + QString::number(remaining_space).toStdString() + 
                       " bytes (reserved: " + QString::number(RESERVE_SPACE).toStdString() + " bytes)", LogLevel::WARNING, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("CaptureImageSaveStatus:USB-NoSpace");
            return 1;
        }
        
        // 创建目录（使用sudo）- 在空间检查通过后
        // 安全检查：避免在 /media/quarcs 路径下创建任何文件夹，避免被错误识别为U盘
        QString normalizedPath = QDir(dirPathToCreate).absolutePath();
        
        // 检查路径是否在 /media/quarcs 下
        user = qEnvironmentVariable("USER");
        if (normalizedPath.startsWith("/run/media/" + user))
        {
            // 提取 /media/quarcs/ 之后的部分
            QString pathAfterMedia = normalizedPath.mid(14); // 去掉 "/media/quarcs/"
            
            // 检查路径格式：应该是 /media/quarcs/某个U盘名/...
            int firstSlash = pathAfterMedia.indexOf('/');
            if (firstSlash > 0)
            {
                QString usbName = pathAfterMedia.left(firstSlash);
                // 检查这个U盘名是否在映射表中（有效的U盘挂载点）
                if (!usbMountPointsMap.contains(usbName))
                {
                    QString message = QString("%1 | Security check failed: Attempting to create directory in /run/media/%2 but USB name '%3' not found in mount points map. Path: %4")
                                        .arg(functionName)
                                        .arg(user)
                                        .arg(usbName)
                                        .arg(dirPathToCreate);

                    Logger::Log(message.toStdString(), LogLevel::ERROR, DeviceType::MAIN);

                    emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
                    return 1;
                }
                // 验证路径确实在U盘挂载点下
                QString expectedMountPoint = "/run/media/"+ user + "/" + usbName;
                if (!normalizedPath.startsWith(expectedMountPoint))
                {
                    QString message = QString("%1 | Security check failed: Path does not match expected mount point. Path: '%2' , Expected mount point: %3")
                                    .arg(functionName)
                                    .arg(dirPathToCreate)
                                    .arg(expectedMountPoint);

                    Logger::Log(message.toStdString(), LogLevel::ERROR, DeviceType::MAIN);

                    emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
                    return 1;
                }
            }
            else
            {
                // 路径格式不正确，可能是直接在 /media/quarcs/ 下创建文件夹
                QString message = QString("%1 | Security check failed: Invalid path format in /run/media/%2. Path: '%3'")
                                    .arg(functionName)
                                    .arg(user)
                                    .arg(dirPathToCreate);

                Logger::Log(message.toStdString(), LogLevel::ERROR, DeviceType::MAIN);

                emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
                return 1;
            }
        }
        // 额外检查：确保路径不是直接在 /media/quarcs 下（没有子目录）
        else if (normalizedPath == "/run/media/" + user)
        {
            QString message = QString(" | Security check failed: Attempting to create directory directly at /run/media/%1. Path: '%2'")
                                .arg(user)
                                .arg(dirPathToCreate);

            Logger::Log(message.toStdString(), LogLevel::ERROR, DeviceType::MAIN);

            emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
            return 1;
        }
        
        //const QString password = "quarcs";
        QProcess mkdirProcess;
        //mkdirProcess.start("sudo", {"-S", "mkdir", "-p", dirPathToCreate});
        mkdirProcess.start("mkdir", {"-p", dirPathToCreate});
        // if (!mkdirProcess.waitForStarted() || !mkdirProcess.write((password + "\n").toUtf8()))
        // {
        //     Logger::Log(functionName.toStdString() + " | Failed to create directory: " + dirPathToCreate.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
        //     emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
        //     return 1;
        // }
        mkdirProcess.closeWriteChannel();
        mkdirProcess.waitForFinished(-1);
    }
    else
    {
        // 默认位置：先检查空间（在创建目录之前）
        QString localPath = QString::fromStdString(ImageSaveBasePath);
        Logger::Log(functionName.toStdString() + " | Local save mode | checking local path: " + localPath.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        long long remaining_space = getUSBSpace(localPath);
        if (remaining_space == -1 || remaining_space <= 0)
        {
            Logger::Log(functionName.toStdString() + " | Local storage has no available space.", LogLevel::WARNING, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("CaptureImageSaveStatus:NoSpace");
            return 1;
        }
        
        // 预留至少100MB的缓冲空间，避免写入时空间不足
        const long long RESERVE_SPACE = 100 * 1024 * 1024; // 100MB
        long long available_space = remaining_space - RESERVE_SPACE;
        if (available_space < 0)
        {
            available_space = 0;
        }
        
        // 检查空间是否足够（文件大小必须小于可用空间，已预留缓冲）
        if (fileSize > available_space)
        {
            Logger::Log(functionName.toStdString() + " | Insufficient local storage space. Required: " + QString::number(fileSize).toStdString() + 
                       " bytes, Available: " + QString::number(remaining_space).toStdString() + 
                       " bytes (reserved: " + QString::number(RESERVE_SPACE).toStdString() + " bytes)", LogLevel::WARNING, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("CaptureImageSaveStatus:NoSpace");
            return 1;
        }
        
        // 创建目录 - 在空间检查通过后
        if (createLocalDirectoryFunc)
        {
            createLocalDirectoryFunc();
        }
    }
    
    return 0;
}

int MainWindow::saveImageFile(const QString &sourcePath, 
                              const QString &destinationPath,
                              const QString &functionName,
                              bool isUSBSave)
{
    if (isUSBSave)
    {
        // U盘保存使用sudo cp命令
        //const QString password = "quarcs";
        QProcess cpProcess;
        // cpProcess.start("sudo", {"-S", "cp", sourcePath, destinationPath});
        cpProcess.start("cp", {sourcePath, destinationPath});
        // if (!cpProcess.waitForStarted() || !cpProcess.write((password + "\n").toUtf8()))
        // {
        //     Logger::Log(functionName.toStdString() + " | Failed to execute sudo cp command.", LogLevel::WARNING, DeviceType::MAIN);
        //     emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
        //     return 1;
        // }
        cpProcess.closeWriteChannel();
        cpProcess.waitForFinished(-1);
        
        if (cpProcess.exitCode() != 0)
        {
            QByteArray stderrOutput = cpProcess.readAllStandardError();
            Logger::Log(functionName.toStdString() + " | Failed to copy file to USB: " + QString::fromUtf8(stderrOutput).toStdString(), LogLevel::WARNING, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
            return 1;
        }
        
        Logger::Log(functionName.toStdString() + " | File saved to USB: " + destinationPath.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    }
    else
    {
        // 默认位置保存使用普通文件操作
        // 将相对路径转换为绝对路径
        QString absoluteDestinationPath = destinationPath;
        if (!QDir::isAbsolutePath(destinationPath))
        {
            absoluteDestinationPath = QDir::currentPath() + "/" + destinationPath;
            Logger::Log(functionName.toStdString() + " | Converted relative path to absolute: " + destinationPath.toStdString() + " -> " + absoluteDestinationPath.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        }
        const char *destinationPathChar = absoluteDestinationPath.toUtf8().constData();

        // 确保目标目录存在
        std::filesystem::path destPath(destinationPathChar);
        std::filesystem::path destDir = destPath.parent_path();
        if (!destDir.empty())
        {
            if (!std::filesystem::exists(destDir))
            {
                try {
                    if (!std::filesystem::create_directories(destDir))
                    {
                        Logger::Log(functionName.toStdString() + " | Failed to create destination directory: " + destDir.string(), LogLevel::ERROR, DeviceType::MAIN);
                        emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
                        return 1;
                    }
                    Logger::Log(functionName.toStdString() + " | Created destination directory: " + destDir.string(), LogLevel::INFO, DeviceType::MAIN);
                } catch (const std::filesystem::filesystem_error& e) {
                    Logger::Log(functionName.toStdString() + " | Exception creating directory: " + std::string(e.what()), LogLevel::ERROR, DeviceType::MAIN);
                    emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
                    return 1;
                }
            }
            else
            {
                // 检查目录是否可写
                try {
                    std::filesystem::perms dirPerms = std::filesystem::status(destDir).permissions();
                    if ((dirPerms & std::filesystem::perms::owner_write) == std::filesystem::perms::none)
                    {
                        Logger::Log(functionName.toStdString() + " | Destination directory is not writable: " + destDir.string(), LogLevel::ERROR, DeviceType::MAIN);
                        emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
                        return 1;
                    }
                } catch (const std::filesystem::filesystem_error& e) {
                    Logger::Log(functionName.toStdString() + " | Exception checking directory permissions: " + std::string(e.what()), LogLevel::WARNING, DeviceType::MAIN);
                }
            }
        }

        // 检查目标文件是否已存在（处理竞态条件）
        if (std::filesystem::exists(destPath))
        {
            Logger::Log(functionName.toStdString() + " | Target file already exists, attempting to remove: " + std::string(destinationPathChar), LogLevel::WARNING, DeviceType::MAIN);
            try {
                if (!std::filesystem::remove(destPath))
                {
                    Logger::Log(functionName.toStdString() + " | Failed to remove existing file: " + std::string(destinationPathChar), LogLevel::ERROR, DeviceType::MAIN);
                    emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
                    return 1;
                }
                Logger::Log(functionName.toStdString() + " | Removed existing file: " + std::string(destinationPathChar), LogLevel::INFO, DeviceType::MAIN);
            } catch (const std::filesystem::filesystem_error& e) {
                Logger::Log(functionName.toStdString() + " | Exception removing existing file: " + std::string(e.what()), LogLevel::ERROR, DeviceType::MAIN);
                emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
                return 1;
            }
        }

        std::ifstream sourceFile(sourcePath.toUtf8().constData(), std::ios::binary);
        if (!sourceFile.is_open())
        {
            Logger::Log(functionName.toStdString() + " | Unable to open source file: " + sourcePath.toStdString(), LogLevel::ERROR, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
            return 1;
        }

        std::ofstream destinationFile(destinationPathChar, std::ios::binary | std::ios::trunc);
        if (!destinationFile.is_open())
        {
            std::string dirInfo = "unknown";
            try {
                if (std::filesystem::exists(destDir))
                {
                    bool writable = (std::filesystem::status(destDir).permissions() & std::filesystem::perms::owner_write) != std::filesystem::perms::none;
                    dirInfo = "exists: yes, writable: " + std::string(writable ? "yes" : "no");
                }
                else
                {
                    dirInfo = "exists: no";
                }
            } catch (...) {
                dirInfo = "exists: unknown (exception)";
            }
            Logger::Log(functionName.toStdString() + " | Unable to create or open target file: " + std::string(destinationPathChar) + 
                       " | " + dirInfo, 
                       LogLevel::ERROR, DeviceType::MAIN);
            sourceFile.close();
            emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
            return 1;
        }

        destinationFile << sourceFile.rdbuf();

        sourceFile.close();
        destinationFile.close();
        
        // 验证文件是否成功写入
        if (!std::filesystem::exists(destPath))
        {
            Logger::Log(functionName.toStdString() + " | File write completed but file does not exist: " + std::string(destinationPathChar), LogLevel::ERROR, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("CaptureImageSaveStatus:Failed");
            return 1;
        }
        
        Logger::Log(functionName.toStdString() + " | File saved successfully to: " + std::string(destinationPathChar) + 
                   " | File size: " + std::to_string(std::filesystem::file_size(destPath)) + " bytes", 
                   LogLevel::INFO, DeviceType::MAIN);
    }
    
    return 0;
}
void MainWindow::getClientSettings()
{

    std::string fileName = "config/config.ini";

    std::unordered_map<std::string, std::string> config;

    Tools::readClientSettings(fileName, config);

    Logger::Log("getClientSettings | Current Config:", LogLevel::INFO, DeviceType::MAIN);
    for (const auto &pair : config)
    {
        Logger::Log("getClientSettings | " + pair.first + " = " + pair.second, LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConfigureRecovery:" + QString::fromStdString(pair.first) + ":" + QString::fromStdString(pair.second));

        if (pair.first == "Coordinates")
        {
            QStringList coordinates = QString::fromStdString(pair.second).split(",");
            if (coordinates.size() >= 2)
            {
                observatorylongitude = coordinates[1].toDouble();
                observatorylatitude = coordinates[0].toDouble();
            }
        }

    }
    Logger::Log("getClientSettings finish!", LogLevel::INFO, DeviceType::MAIN);
}

void MainWindow::setClientSettings(QString ConfigName, QString ConfigValue)
{

    Logger::Log("setClientSettings start ...", LogLevel::INFO, DeviceType::MAIN);
    std::string fileName = "config/config.ini";

    std::unordered_map<std::string, std::string> config;

    config[ConfigName.toStdString()] = ConfigValue.toStdString();

    Logger::Log("setClientSettings | Save Client Setting:" + ConfigName.toStdString() + " = " + ConfigValue.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    Tools::saveClientSettings(fileName, config);
    if (ConfigName == "FocalLength")
    {
        glFocalLength = ConfigValue.toDouble();
    }
    Logger::Log("setClientSettings finish!", LogLevel::INFO, DeviceType::MAIN);
}

void MainWindow::getConnectedDevices()
{
    Logger::Log("getConnectedDevices start ...", LogLevel::INFO, DeviceType::MAIN);
    QString deviceType;
    bool isConnect;
    for (int i = 0; i < systemdevicelist.system_devices.size(); i++)
    {
        deviceType = systemdevicelist.system_devices[i].Description;
        isConnect = systemdevicelist.system_devices[i].isConnect;
        if (deviceType != "" && isConnect)
        {
            emit wsThread->sendMessageToClient("AddDeviceType:" + deviceType);
        }
    }

    for (int i = 0; i < indi_Client->GetDeviceCount(); i++)
    {
        if (indi_Client->GetDeviceFromList(i)->isConnected())
        {
            emit wsThread->sendMessageToClient("DeviceToBeAllocated:Device:" + QString::number(i) + ":" + QString::fromUtf8(indi_Client->GetDeviceFromList(i)->getDeviceName()));
        }
    }

    for (int i = 0; i < ConnectedDevices.size(); i++)
    {
        Logger::Log("getConnectedDevices | Device[" + std::to_string(i) + "]: " + ConnectedDevices[i].DeviceName.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConnectSuccess:" + ConnectedDevices[i].DeviceType + ":" + ConnectedDevices[i].DeviceName);

        if (ConnectedDevices[i].DeviceType == "MainCamera" && dpMainCamera != NULL)
        {
            emit wsThread->sendMessageToClient("MainCameraSize:" + QString::number(glMainCCDSizeX) + ":" + QString::number(glMainCCDSizeY));
            emit wsThread->sendMessageToClient("MainCameraOffsetRange:" + QString::number(glOffsetMin) + ":" + QString::number(glOffsetMax) + ":" + QString::number(glOffsetValue));
            emit wsThread->sendMessageToClient("MainCameraGainRange:" + QString::number(glGainMin) + ":" + QString::number(glGainMax) + ":" + QString::number(glGainValue));

            QString CFWname;
            indi_Client->getCFWSlotName(dpMainCamera, CFWname);
            if (CFWname != "")
            {
                Logger::Log("getConnectedDevices | get CFW Slot Name: " + CFWname.toStdString(), LogLevel::INFO, DeviceType::MAIN);
                emit wsThread->sendMessageToClient("ConnectSuccess:CFW:" + CFWname + " (on camera)");
                isFilterOnCamera = true;

                int min, max, pos;
                indi_Client->getCFWPosition(dpMainCamera, pos, min, max);
                Logger::Log("getConnectedDevices | getCFWPosition: " + std::to_string(min) + ", " + std::to_string(max) + ", " + std::to_string(pos), LogLevel::INFO, DeviceType::MAIN);
                emit wsThread->sendMessageToClient("CFWPositionMax:" + QString::number(max));
            }
        }
    }
    Logger::Log("getConnectedDevices finish!", LogLevel::INFO, DeviceType::MAIN);
}

void MainWindow::clearConnectedDevices()
{
    ConnectedDevices.clear();
}

void MainWindow::getStagingImage()
{
    if (isStagingImage && SavedImage != "" && isFileExists(QString::fromStdString(vueImagePath + SavedImage)) && isFocusLoopShooting)
    {
        Logger::Log("getStagingImage | ready to upload image: " + SavedImage, LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("SaveBinSuccess:" + QString::fromStdString(SavedImage));
    }
    else
    {
        Logger::Log("getStagingImage | no image to upload", LogLevel::INFO, DeviceType::MAIN);
    }
}

bool MainWindow::isFileExists(const QString &filePath)
{
    QFile file(filePath);
    if (file.exists())
    {
        Logger::Log("isFileExists | file exists: " + filePath.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    }
    else
    {
        Logger::Log("isFileExists | file does not exist: " + filePath.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    }
    return file.exists();
}

void MainWindow::getStagingScheduleData()
{
    if (isStagingScheduleData)
    {
        emit wsThread->sendMessageToClient(StagingScheduleData);
    }

    // 将当前调度列表中的进度同步给前端，便于页面刷新后恢复每一行的执行进度
    for (int i = 0; i < m_scheduList.size(); ++i)
    {
        int progress = m_scheduList[i].progress;
        if (progress < 0)
        {
            progress = 0;
        }
        else if (progress > 100)
        {
            progress = 100;
        }

        // 仅对已有进度的行进行同步，避免干扰尚未使用的默认行
        if (progress > 0)
        {
            emit wsThread->sendMessageToClient(
                "UpdateScheduleProcess:" +
                QString::number(i) + ":" +
                QString::number(progress));
        }
    }

    // 无论是否有暂存数据，都向前端同步当前计划运行状态
    emit wsThread->sendMessageToClient(
        QString("ScheduleRunning:%1").arg(isScheduleRunning ? "true" : "false"));
}

void MainWindow::getStagingGuiderData()
{
    int dataSize = glPHD_rmsdate.size();
    int startIdx = dataSize > 50 ? dataSize - 50 : 0;

    for (int i = startIdx; i < dataSize; i++)
    {
        emit wsThread->sendMessageToClient("AddLineChartData:" + QString::number(i) + ":" + QString::number(glPHD_rmsdate[i].x()) + ":" + QString::number(glPHD_rmsdate[i].y()));
        emit wsThread->sendMessageToClient("AddScatterChartData:" + QString::number(glPHD_rmsdate[i].x()) + ":" + QString::number(-glPHD_rmsdate[i].y()));
        if (i > 50)
        {
            emit wsThread->sendMessageToClient("SetLineChartRange:" + QString::number(i - 50) + ":" + QString::number(i));
        }
    }
}

int MainWindow::MoveFileToUSB()
{
    qDebug("MoveFileToUSB");
}

void MainWindow::solveCurrentPosition()
{
    if (solveCurrentPositionTimer.isActive())
    {
        Logger::Log("solveCurrentPosition | SolveCurrentPosition is already running...", LogLevel::INFO, DeviceType::MAIN);
        return;
    }
    // 停止之前的定时器
    solveCurrentPositionTimer.stop();
    disconnect(&solveCurrentPositionTimer, &QTimer::timeout, nullptr, nullptr);
    // 判断解析图像路径下是否有图片
    if (isFileExists(QString::fromStdString(SolveImageFileName.toStdString())))
    {
        // 设置定时器为单次触发
        solveCurrentPositionTimer.setSingleShot(true);
        // 开始解析图像
        Tools::PlateSolve(SolveImageFileName, glFocalLength, glCameraSize_width, glCameraSize_height, false);
        // 连接解析完成信号到处理函数，处理解析完成后的逻辑
        connect(&solveCurrentPositionTimer, &QTimer::timeout, [this]()
        {
            if (Tools::isSolveImageFinish())  // 检查图像解析是否完成
            {
                SloveResults result = Tools::ReadSolveResult(SolveImageFileName, glMainCCDSizeX, glMainCCDSizeY);  // 读取解析结果
                if (result.RA_Degree == -1 && result.DEC_Degree == -1)
                {
                    Logger::Log("solveCurrentPosition | Solve image failed...", LogLevel::INFO, DeviceType::MAIN);
                    emit wsThread->sendMessageToClient("SolveCurrentPosition:failed");
                    solveCurrentPositionTimer.stop();
                    disconnect(&solveCurrentPositionTimer, &QTimer::timeout, nullptr, nullptr);
                    return;
                }
                else
                {
                    emit wsThread->sendMessageToClient("SolveCurrentPosition:succeeded:" + QString::number(result.RA_Degree) + ":" + QString::number(result.DEC_Degree)+":"+QString::number(result.RA_0)+":"+QString::number(result.DEC_0)+":"+QString::number(result.RA_1)+":"+QString::number(result.DEC_1)+":"+QString::number(result.RA_2)+":"+QString::number(result.DEC_2)+":"+QString::number(result.RA_3)+":"+QString::number(result.DEC_3));
                    solveCurrentPositionTimer.stop();
                    disconnect(&solveCurrentPositionTimer, &QTimer::timeout, nullptr, nullptr);
                    return;
                }
            }
            else
            {
                solveCurrentPositionTimer.start(1000);
            }
        });
        solveCurrentPositionTimer.start(1000);
    }
    else
    {
        Logger::Log("solveCurrentPosition | SolveImageFileName: " + SolveImageFileName.toStdString() + " does not exist.", LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("SolveCurrentPosition:failed");
        solveCurrentPositionTimer.stop();
        return;
    }
}

void MainWindow::TelescopeControl_SolveSYNC()
{
    // 在函数开始时断开之前的连接
    disconnect(&captureTimer, &QTimer::timeout, nullptr, nullptr);
    disconnect(&solveTimer, &QTimer::timeout, nullptr, nullptr);

    // 停止之前的定时器
    captureTimer.stop();
    solveTimer.stop();

    if (dpMainCamera == NULL)
    {
        emit wsThread->sendMessageToClient("MainCameraNotConnect");
        return;
    }
    Logger::Log("TelescopeControl_SolveSYNC start ...", LogLevel::INFO, DeviceType::MAIN);
    if (glMainCameraStatu == "Exposuring" || isFocusLoopShooting == true)
    {
        Logger::Log("TelescopeControl_SolveSYNC | Camera is not idle.glMainCameraStatu:" + glMainCameraStatu.toStdString() + ", isFocusLoopShooting:" + std::to_string(isFocusLoopShooting), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("CameraNotIdle");
        return;
    }

    double Ra_Hour;
    double Dec_Degree;

    if (dpMount != NULL)
    {
        indi_Client->getTelescopeRADECJNOW(dpMount, Ra_Hour, Dec_Degree); // 获取当前望远镜的赤经和赤纬
    }
    else
    {
        Logger::Log("TelescopeControl_SolveSYNC | No Mount Connect.", LogLevel::INFO, DeviceType::MAIN);
        return; // 如果望远镜未连接，记录日志并退出
    }
    isSolveSYNC = true;
    double Ra_Degree = Tools::HourToDegree(Ra_Hour); // 将赤经从小时转换为度

    Logger::Log("TelescopeControl_SolveSYNC | CurrentRa(Degree):" + std::to_string(Ra_Degree) + "," + "CurrentDec(Degree):" + std::to_string(Dec_Degree), LogLevel::INFO, DeviceType::MAIN);
    isSavePngSuccess = false;
    INDI_Capture(1000); // 拍摄1秒曝光进行解析同步

    captureTimer.setSingleShot(true);

    // 连接拍摄定时器的超时信号到处理函数，处理拍摄完成后的逻辑
    connect(&captureTimer, &QTimer::timeout, [this](){
        // 如果需要中止拍摄和解算，则执行中止操作并返回
        if (EndCaptureAndSolve)
        {
            EndCaptureAndSolve = false;
            INDI_AbortCapture();
            Logger::Log("TelescopeControl_SolveSYNC | End Capture And Solve!!!", LogLevel::INFO, DeviceType::MAIN);
            isSolveSYNC = false;
            emit wsThread->sendMessageToClient("SolveImagefailed"); 
            return;
        }
        Logger::Log("TelescopeControl_SolveSYNC | WaitForShootToComplete ..." , LogLevel::INFO, DeviceType::MAIN);
  
        // 检查拍摄是否完成
        if (isSavePngSuccess) 
        {
            // 停止拍摄定时器，表示拍摄任务完成
            captureTimer.stop();

            // 开始进行图像解析
            Tools::PlateSolve(SolveImageFileName, glFocalLength, glCameraSize_width, glCameraSize_height, false);

            solveTimer.setSingleShot(true);  // 设置解析定时器为单次触发

            connect(&solveTimer, &QTimer::timeout, [this]()
            {
                // 检查解析进程是否已结束
                bool solveProcessFinished = !Tools::isPlateSolveInProgress();
                
                if (Tools::isSolveImageFinish())  // 检查图像解析是否成功完成
                {
                    SloveResults result = Tools::ReadSolveResult(SolveImageFileName, glMainCCDSizeX, glMainCCDSizeY);  // 读取解析结果
                    Logger::Log("TelescopeControl_SolveSYNC | Plate Solve Result(RA_Degree, DEC_Degree):" + std::to_string(result.RA_Degree) + ", " + std::to_string(result.DEC_Degree), LogLevel::INFO, DeviceType::MAIN);

                    if (result.RA_Degree == -1 && result.DEC_Degree == -1)
                    {
                        Logger::Log("TelescopeControl_SolveSYNC | Solve image failed...", LogLevel::INFO, DeviceType::MAIN);
                        Logger::Log("TelescopeControl_SolveSYNC | SolveImageFileName: " + SolveImageFileName.toStdString(), LogLevel::INFO, DeviceType::MAIN);
                        Logger::Log("TelescopeControl_SolveSYNC | mainCameraSaveFailedParse: " + std::string(mainCameraSaveFailedParse ? "true" : "false"), LogLevel::INFO, DeviceType::MAIN);
                        if (mainCameraSaveFailedParse)
                        {
                            int saveResult = solveFailedImageSave(SolveImageFileName);
                            if (saveResult == 0)
                            {
                                Logger::Log("TelescopeControl_SolveSYNC | Failed solve image saved successfully", LogLevel::INFO, DeviceType::MAIN);
                            }
                            else
                            {
                                Logger::Log("TelescopeControl_SolveSYNC | Failed to save failed solve image, error code: " + std::to_string(saveResult), LogLevel::WARNING, DeviceType::MAIN);
                            }
                        }
                        else
                        {
                            Logger::Log("TelescopeControl_SolveSYNC | mainCameraSaveFailedParse is disabled, skipping save", LogLevel::INFO, DeviceType::MAIN);
                        }
                        emit wsThread->sendMessageToClient("SolveImagefailed");  // 发送解析失败的消息
                        isSolveSYNC = false;
                        solveTimer.stop();  // 停止定时器
                    }
                    else
                    {
                        if (dpMount != NULL)
                        {
                            INDI::PropertyNumber property = NULL;
                            Logger::Log("TelescopeControl_SolveSYNC | syncTelescopeJNow | start", LogLevel::INFO, DeviceType::MAIN);
                            QString action = "SYNC";
                            bool isTrack = false;
                            indi_Client->getTelescopeTrackEnable(dpMount, isTrack);
                            if (!isTrack)
                            {
                                indi_Client->setTelescopeTrackEnable(dpMount, true);
                            }
                            emit wsThread->sendMessageToClient("TelescopeTrack:ON");

                            // 解析结果 RA/DEC 为“度”，下发到 INDI 前需要转换为 RA 小时制
                            double solvedRaHour = Tools::DegreeToHour(result.RA_Degree);
                            double solvedDecDeg = result.DEC_Degree;

                            // 同步望远镜的当前位置到目标位置（JNOW, RA:hour / DEC:deg）
                            uint32_t syncResult = indi_Client->syncTelescopeJNow(dpMount, solvedRaHour, solvedDecDeg);
                            if (syncResult != QHYCCD_SUCCESS)
                            {
                                Logger::Log("TelescopeControl_SolveSYNC | syncTelescopeJNow failed",
                                            LogLevel::ERROR, DeviceType::MAIN);
                                emit wsThread->sendMessageToClient("SolveImagefailed");
                            }
                            else
                            {
                                Logger::Log("TelescopeControl_SolveSYNC | syncTelescopeJNow | end", LogLevel::INFO, DeviceType::MAIN);
                                // Logger::Log("TelescopeControl_SolveSYNC | DegreeToHour:" + std::to_string(Tools::DegreeToHour(result.RA_Degree)) + "DEC_Degree:" + std::to_string(result.DEC_Degree), LogLevel::INFO, DeviceType::MAIN);

                                // indi_Client->setTelescopeRADECJNOW(dpMount, Tools::DegreeToHour(result.RA_Degree), result.DEC_Degree);  // 设置望远镜的目标位置
                                // Logger::Log("TelescopeControl_SolveSYNC | syncTelescopeJNow | end", LogLevel::INFO, DeviceType::MAIN);
                                // double a, b;
                                // indi_Client->getTelescopeRADECJNOW(dpMount, a, b);  // 获取望远镜的当前位置
                                // Logger::Log("TelescopeControl_SolveSYNC | Get_RA_Hour:" + std::to_string(a) + "Get_DEC_Degree:" + std::to_string(b), LogLevel::INFO, DeviceType::MAIN);
                                emit wsThread->sendMessageToClient("SolveImageSucceeded");
                            }

                            isSolveSYNC = false;
                            solveTimer.stop();  // 停止定时器
                        }
                        else
                        {
                            Logger::Log("TelescopeControl_SolveSYNC | No Mount Connect.", LogLevel::INFO, DeviceType::MAIN);
                            emit wsThread->sendMessageToClient("SolveImagefailed");  // 发送解析失败的消息
                            isSolveSYNC = false;
                            solveTimer.stop();  // 停止定时器
                            return;  // 如果望远镜未连接，记录日志并退出
                        }
                    }
                }
                else if (solveProcessFinished)
                {
                    // 解析进程已结束但未成功完成（退出码非0的情况）
                    Logger::Log("TelescopeControl_SolveSYNC | Solve process finished but failed (exit code != 0)", LogLevel::ERROR, DeviceType::MAIN);
                    Logger::Log("TelescopeControl_SolveSYNC | SolveImageFileName: " + SolveImageFileName.toStdString(), LogLevel::INFO, DeviceType::MAIN);
                    Logger::Log("TelescopeControl_SolveSYNC | mainCameraSaveFailedParse: " + std::string(mainCameraSaveFailedParse ? "true" : "false"), LogLevel::INFO, DeviceType::MAIN);
                    if (mainCameraSaveFailedParse)
                    {
                        int saveResult = solveFailedImageSave(SolveImageFileName);
                        if (saveResult == 0)
                        {
                            Logger::Log("TelescopeControl_SolveSYNC | Failed solve image saved successfully", LogLevel::INFO, DeviceType::MAIN);
                        }
                        else
                        {
                            Logger::Log("TelescopeControl_SolveSYNC | Failed to save failed solve image, error code: " + std::to_string(saveResult), LogLevel::WARNING, DeviceType::MAIN);
                        }
                    }
                    else
                    {
                        Logger::Log("TelescopeControl_SolveSYNC | mainCameraSaveFailedParse is disabled, skipping save", LogLevel::INFO, DeviceType::MAIN);
                    }
                    emit wsThread->sendMessageToClient("SolveImagefailed");  // 发送解析失败的消息
                    isSolveSYNC = false;
                    solveTimer.stop();  // 停止定时器
                }
                else 
                {
                    solveTimer.start(1000);  // 如果解析未完成，重新启动定时器继续等待
                } 
            });

            solveTimer.start(1000);  // 启动解析定时器

        } 
        else 
        {
            // 如果拍摄未完成，重新启动拍摄定时器，继续等待
            captureTimer.start(1000);
        } });
    captureTimer.start(1000);
}

LocationResult MainWindow::TelescopeControl_GetLocation()
{
    if (dpMount != NULL)
    {
        LocationResult result;

        indi_Client->getLocation(dpMount, result.latitude_degree, result.longitude_degree, result.elevation);

        return result;
    }
}

QDateTime MainWindow::TelescopeControl_GetTimeUTC()
{
    if (dpMount != NULL)
    {
        QDateTime result;

        indi_Client->getTimeUTC(dpMount, result);

        return result;
    }
}

SphericalCoordinates MainWindow::TelescopeControl_GetRaDec()
{
    if (dpMount != NULL)
    {
        SphericalCoordinates result;
        double RA_HOURS, DEC_DEGREE;
        indi_Client->getTelescopeRADECJNOW(dpMount, RA_HOURS, DEC_DEGREE);
        result.ra = Tools::HourToDegree(RA_HOURS);
        result.dec = DEC_DEGREE;

        return result;
    }
}

void MainWindow::MountGoto(double Ra_Hour, double Dec_Degree)
{
    Logger::Log("MountGoto start ...", LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("MountGoto | RaDec(Hour):" + std::to_string(Ra_Hour) + "," + std::to_string(Dec_Degree), LogLevel::INFO, DeviceType::MAIN);

    // 在执行 GOTO 之前，如当前处于导星状态，则暂时停止导星，待转动完成后再恢复
    pauseGuidingBeforeMountMove();

    // 停止和清理先前的计时器
    telescopeTimer.stop();
    telescopeTimer.disconnect();

    TelescopeControl_Goto(Ra_Hour, Dec_Degree);

    sleep(2); // 赤道仪的状态更新有一定延迟

    // 启动等待赤道仪转动的定时器
    telescopeTimer.setSingleShot(true);

    connect(&telescopeTimer, &QTimer::timeout, [this, Ra_Hour, Dec_Degree]()
            {
        if (WaitForTelescopeToComplete()) 
        {
            telescopeTimer.stop();  // 转动完成时停止定时器
            Logger::Log("MountGoto | Mount Goto Complete!", LogLevel::INFO, DeviceType::MAIN);

            // 如果本次 GOTO 之前处于导星状态，则在赤道仪转动完成后恢复导星
            resumeGuidingAfterMountMove();
            if (GotoThenSolve) // 判断是否进行解算
            {
                Logger::Log("MountGoto | Goto Then Solve!", LogLevel::INFO, DeviceType::MAIN);
                // 启动一次解算同步流程
                isSolveSYNC = true;
                TelescopeControl_SolveSYNC(); // 开始拍摄解析
                
                if (GotoOlveTimer != nullptr)
                {
                    delete GotoOlveTimer;
                    GotoOlveTimer = nullptr;
                }
                GotoOlveTimer = new QTimer();
                GotoOlveTimer->setSingleShot(true);
                connect(GotoOlveTimer, &QTimer::timeout, [this, Ra_Hour, Dec_Degree]()
                {
                    if (!isSolveSYNC)
                    {
                        GotoOlveTimer->stop();
                        Logger::Log("MountGoto | Goto Then Solve Complete!", LogLevel::INFO, DeviceType::MAIN);
                        // 解算同步完成后，仅再执行一次 Goto 回到目标坐标
                        TelescopeControl_Goto(Ra_Hour, Dec_Degree);
                    }else{
                        GotoOlveTimer->start(1000);
                    }
                });
                GotoOlveTimer->start(1000);
            }
        } 
        else 
        {
            telescopeTimer.start(1000);  // 继续等待
        } });

    telescopeTimer.start(1000);

    Logger::Log("MountGoto finish!", LogLevel::INFO, DeviceType::MAIN);
}

void MainWindow::MountOnlyGoto(double Ra_Hour, double Dec_Degree)
{
    if (dpMount == NULL)
    {
        Logger::Log("MountOnlyGoto | No Mount Connect.", LogLevel::ERROR, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("MountOnlyGotoFailed:No Mount Connect");  // 发送转到失败的消息
        return;
    }
    if (Ra_Hour < 0 || Ra_Hour > 24 || Dec_Degree < -90 || Dec_Degree > 90)
    {
        Logger::Log("MountOnlyGoto | Invalid RaDec(Hour):" + std::to_string(Ra_Hour) + "," + std::to_string(Dec_Degree), LogLevel::ERROR, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("MountOnlyGotoFailed:Invalid RaDec(Hour)");  // 发送转到失败的消息
        return;
    }
    if (indi_Client->mountState.isMovingNow())
    {
        Logger::Log("MountOnlyGoto | Mount is Moving.", LogLevel::ERROR, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("MountOnlyGotoFailed:Mount is Moving");  // 发送转到失败的消息
        return;
    }
    Logger::Log("MountOnlyGoto start ...", LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("MountOnlyGoto | RaDec(Hour):" + std::to_string(Ra_Hour) + "," + std::to_string(Dec_Degree), LogLevel::INFO, DeviceType::MAIN);

    // 在执行 Goto 之前，如当前处于导星状态，则暂时停止导星，待转动完成后再恢复
    pauseGuidingBeforeMountMove();

    // 停止和清理先前的计时器
    telescopeTimer.stop();
    telescopeTimer.disconnect();

    TelescopeControl_Goto(Ra_Hour, Dec_Degree); // 转到目标位置

    sleep(2); // 赤道仪的状态更新有一定延迟

    // 启动等待赤道仪转动的定时器
    telescopeTimer.setSingleShot(true);

    connect(&telescopeTimer, &QTimer::timeout, [this, Ra_Hour, Dec_Degree]()
    {
        if (WaitForTelescopeToComplete())
        {
            telescopeTimer.stop();  // 转动完成时停止定时器
            Logger::Log("MountOnlyGoto | Mount Only Goto Complete!", LogLevel::INFO, DeviceType::MAIN);
            // 如果本次 Goto 之前处于导星状态，则在赤道仪转动完成后恢复导星
            resumeGuidingAfterMountMove();
            emit wsThread->sendMessageToClient("MountOnlyGotoSuccess");  // 发送转到成功消息
        }
        else
        {
            telescopeTimer.start(1000);  // 继续等待赤道仪转动
        }
    });
    telescopeTimer.start(1000);

    Logger::Log("MountOnlyGoto finish!", LogLevel::INFO, DeviceType::MAIN);

}
void MainWindow::DeleteImage(QStringList DelImgPath)
{
    //std::string password = "quarcs"; // sudo 密码
    for (int i = 0; i < DelImgPath.size(); i++)
    {
        if (i < DelImgPath.size())
        {
            std::ostringstream commandStream;
            //commandStream << "echo '" << password << "' | sudo -S rm -rf \"./" << DelImgPath[i].toStdString() << "\"";
            commandStream << "rm -rf \"./" << DelImgPath[i].toStdString() << "\"";
            std::string command = commandStream.str();

            Logger::Log("DeleteImage | Deleted command:" + QString::fromStdString(command).toStdString(), LogLevel::INFO, DeviceType::MAIN);

            // 执行系统命令删除文件
            int result = system(command.c_str());

            if (result == 0)
            {
                Logger::Log("DeleteImage | Deleted file:" + DelImgPath[i].toStdString(), LogLevel::INFO, DeviceType::MAIN);
            }
            else
            {
                Logger::Log("DeleteImage | Failed to delete file:" + DelImgPath[i].toStdString(), LogLevel::INFO, DeviceType::MAIN);
            }
        }
        else
        {
            Logger::Log("DeleteImage | Index out of range: " + std::to_string(i), LogLevel::WARNING, DeviceType::MAIN);
        }
    }
}

std::string MainWindow::GetAllFile()
{
    Logger::Log("GetAllFile start ...", LogLevel::INFO, DeviceType::MAIN);
    std::string capturePath = ImageSaveBasePath + "/CaptureImage/";
    std::string planPath = ImageSaveBasePath + "/ScheduleImage/";
    std::string solveFailedImagePath = ImageSaveBasePath + "/solveFailedImage/";
    std::string resultString;
    std::string captureString = "CaptureImage{";
    std::string planString = "ScheduleImage{";
    std::string solveFailedImageString = "SolveFailedImage{";

    try
    {
        // 检查并处理 CaptureImage 目录
        if (std::filesystem::exists(capturePath) && std::filesystem::is_directory(capturePath))
        {
            for (const auto &entry : std::filesystem::directory_iterator(capturePath))
            {
                std::string fileName = entry.path().filename().string(); // 获取文件名（包含扩展名）
                captureString += fileName + ";";                         // 拼接为字符串
            }
        }
        else
        {
            Logger::Log("GetAllFile | CaptureImage directory does not exist or is not a directory: " + capturePath, LogLevel::WARNING, DeviceType::MAIN);
        }

        // 检查并处理 ScheduleImage 目录
        if (std::filesystem::exists(planPath) && std::filesystem::is_directory(planPath))
        {
            for (const auto &entry : std::filesystem::directory_iterator(planPath))
            {
                std::string folderName = entry.path().filename().string(); // 获取文件夹名
                planString += folderName + ";";
            }
        }
        else
        {
            Logger::Log("GetAllFile | ScheduleImage directory does not exist or is not a directory: " + planPath, LogLevel::WARNING, DeviceType::MAIN);
        }
        // 检查并处理 solveFailedImage 目录
        if (std::filesystem::exists(solveFailedImagePath) && std::filesystem::is_directory(solveFailedImagePath))
        {
            for (const auto &entry : std::filesystem ::directory_iterator(solveFailedImagePath))
            {
                std::string fileName = entry.path().filename().string(); // 获取文件名（包含扩展名）
                solveFailedImageString += fileName + ";";                // 拼接为字符串
            }
        }
        else
        {
            Logger::Log("GetAllFile | SolveFailedImage directory does not exist or is not a directory: " + solveFailedImagePath, LogLevel::WARNING, DeviceType::MAIN);
            // solveFailedImageString = "SolveFailedImage{}"; // 如果目录不存在，返回空字符串
        }
    }
    catch (const std::filesystem::filesystem_error &e)
    {
        Logger::Log("GetAllFile | Filesystem error: " + std::string(e.what()), LogLevel::ERROR, DeviceType::MAIN);
    }
    catch (const std::exception &e)
    {
        Logger::Log("GetAllFile | General error: " + std::string(e.what()), LogLevel::ERROR, DeviceType::MAIN);
    }

    resultString = captureString + "}:" + planString + "}:" + solveFailedImageString + '}';
    Logger::Log("GetAllFile finish!", LogLevel::INFO, DeviceType::MAIN);
    return resultString;
}
void MainWindow::GetImageFiles(std::string ImageFolder)
{
    Logger::Log("GetImageFiles start ...", LogLevel::INFO, DeviceType::MAIN);
    std::string basePath = ImageSaveBasePath + "/" + ImageFolder + "/";
    std::string ImageFilesNameString = "";

    try
    {
        // 检查目录是否存在
        if (!std::filesystem::exists(basePath))
        {
            Logger::Log("GetImageFiles | Directory does not exist: " + basePath, LogLevel::WARNING, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("ImageFilesName:");
            Logger::Log("GetImageFiles finish! (Directory not found)", LogLevel::INFO, DeviceType::MAIN);
            return;
        }

        if (!std::filesystem::is_directory(basePath))
        {
            Logger::Log("GetImageFiles | Path is not a directory: " + basePath, LogLevel::WARNING, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("ImageFilesName:");
            Logger::Log("GetImageFiles finish! (Not a directory)", LogLevel::INFO, DeviceType::MAIN);
            return;
        }

        for (const auto &entry : std::filesystem::directory_iterator(basePath))
        {
            std::string fileName = entry.path().filename().string(); // 获取文件名（包含扩展名）
            ImageFilesNameString += fileName + ";";                  // 拼接为字符串
        }
    }
    catch (const std::filesystem::filesystem_error &e)
    {
        Logger::Log("GetImageFiles | Filesystem error: " + std::string(e.what()), LogLevel::ERROR, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ImageFilesName:");
        Logger::Log("GetImageFiles finish! (Filesystem error)", LogLevel::INFO, DeviceType::MAIN);
        return;
    }
    catch (const std::exception &e)
    {
        Logger::Log("GetImageFiles | General error: " + std::string(e.what()), LogLevel::ERROR, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ImageFilesName:");
        Logger::Log("GetImageFiles finish! (General error)", LogLevel::INFO, DeviceType::MAIN);
        return;
    }

    Logger::Log("GetImageFiles | Image Files:" + QString::fromStdString(ImageFilesNameString).toStdString(), LogLevel::INFO, DeviceType::MAIN);
    emit wsThread->sendMessageToClient("ImageFilesName:" + QString::fromStdString(ImageFilesNameString));
    Logger::Log("GetImageFiles finish!", LogLevel::INFO, DeviceType::MAIN);
}



// 解析字符串
QStringList MainWindow::parseString(const std::string &input, const std::string &imgFilePath)
{
    QStringList paths;
    QString baseString;
    size_t pos = input.find('{');
    if (pos != std::string::npos)
    {
        baseString = QString::fromStdString(input.substr(0, pos));
        std::string content = input.substr(pos + 1);
        size_t endPos = content.find('}');
        if (endPos != std::string::npos)
        {
            content = content.substr(0, endPos);

            // 去掉末尾的分号（如果有的话）
            if (!content.empty() && content.back() == ';')
            {
                content.pop_back();
            }

            QStringList parts = QString::fromStdString(content).split(';', Qt::SkipEmptyParts);
            for (const QString &part : parts)
            {
                QString path = QDir::toNativeSeparators(QString::fromStdString(imgFilePath) + "/" + baseString + "/" + part);
                paths.append(path);
            }
        }
    }
    return paths;
}

// 返回 U 盘剩余内存
long long MainWindow::getUSBSpace(const QString &usb_mount_point)
{
    Logger::Log("getUSBSpace start ...", LogLevel::INFO, DeviceType::MAIN);
    struct statvfs stat;
    if (statvfs(usb_mount_point.toUtf8().constData(), &stat) == 0)
    {
        // 使用 f_bavail 而不是 f_bfree，因为 f_bavail 是普通用户实际可用的空间
        // f_bfree 可能包含系统保留的空间，实际用户可能无法使用
        long long free_space = static_cast<long long>(stat.f_bavail) * stat.f_frsize;
        Logger::Log("getUSBSpace | USB Space (available): " + std::to_string(free_space) + " bytes", LogLevel::INFO, DeviceType::MAIN);
        return free_space;
    }
    else
    {
        Logger::Log("getUSBSpace | Failed to obtain the space information of the USB flash drive.", LogLevel::WARNING, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("getUSBFail:Failed to obtain the space information of the USB flash drive.");
        return -1;
    }
}
long long MainWindow::getTotalSize(const QStringList &filePaths)
{
    long long totalSize = 0;
    foreach (QString filePath, filePaths)
    {
        QFileInfo fileInfo(filePath);
        if (fileInfo.exists())
        {
            totalSize += fileInfo.size();
        }
    }
    return totalSize;
}

// 获取文件系统挂载模式
bool MainWindow::isMountReadOnly(const QString &mountPoint)
{
    struct statvfs fsinfo;
    auto mountPointStr = mountPoint.toUtf8().constData();
    Logger::Log("isMountReadOnly | Checking filesystem information for mount point:" + QString::fromUtf8(mountPointStr).toStdString(), LogLevel::INFO, DeviceType::MAIN);

    if (statvfs(mountPointStr, &fsinfo) != 0)
    {
        Logger::Log("isMountReadOnly | Failed to get filesystem information for" + mountPoint.toStdString() + ":" + strerror(errno), LogLevel::WARNING, DeviceType::MAIN);
        emit wsThread->sendMessageToClient(QString("getUSBFail:Failed to get filesystem information for %1, error: %2").arg(mountPoint).arg(strerror(errno)));
        return false;
    }

    Logger::Log("isMountReadOnly | Filesystem flags for" + mountPoint.toStdString() + ":" + std::to_string(fsinfo.f_flag), LogLevel::INFO, DeviceType::MAIN);
    return (fsinfo.f_flag & ST_RDONLY) != 0;
}

// AstroArch use Udisks
// 将文件系统挂载模式更改为读写模式
bool MainWindow::remountReadWrite(const QString &mountPoint, const QString &password)
 {
//     QProcess process;
//     process.start("sudo", {"-S", "mount", "-o", "remount,rw", mountPoint});
//     if (!process.waitForStarted() || !process.write((password + "\n").toUtf8()))
//     {
//         Logger::Log("remountReadWrite | Failed to execute command: sudo mount", LogLevel::WARNING, DeviceType::MAIN);
//         emit wsThread->sendMessageToClient("getUSBFail:Failed to execute command: sudo mount -o remount,rw usb.");
//         return false;
//     }
//     process.closeWriteChannel();
//     process.waitForFinished(-1);
//     return process.exitCode() == 0;
 }

void MainWindow::RemoveImageToUsb(QStringList RemoveImgPath, QString usbName)
{
    QString usb_mount_point = "";
    
    // 如果提供了U盘名，优先使用它从映射表中查找
    if (!usbName.isEmpty() && usbMountPointsMap.contains(usbName))
    {
        usb_mount_point = usbMountPointsMap[usbName];
        Logger::Log("RemoveImageToUsb | Using specified USB from map: " + usbName.toStdString() + " -> " + usb_mount_point.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    }
    
    // 如果上面没有获取到，优先使用 ImageSaveBaseDirectory 指定的U盘路径
    if (usb_mount_point.isEmpty())
    {
        // 使用saveMode判断是否为U盘保存
        bool isUSBSave = (saveMode != "local");
        
        if (isUSBSave && ImageSaveBaseDirectory.contains("/QUARCS_ImageSave"))
        {
            // 从 ImageSaveBaseDirectory 提取U盘挂载点
            usb_mount_point = ImageSaveBaseDirectory;
            usb_mount_point.replace("/QUARCS_ImageSave", "");
            
            // 验证该U盘是否仍然存在且有效
            QStorageInfo storageInfo(usb_mount_point);
            if (!storageInfo.isValid() || !storageInfo.isReady())
            {
                Logger::Log("RemoveImageToUsb | Specified USB path is no longer valid: " + usb_mount_point.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
                usb_mount_point = ""; // 重置，使用下面的逻辑重新获取
            }
            else
            {
                Logger::Log("RemoveImageToUsb | Using USB from ImageSaveBaseDirectory: " + usb_mount_point.toStdString(), LogLevel::INFO, DeviceType::MAIN);
            }
        }
    }
    
    // 如果上面没有获取到，尝试从映射表获取
    if (usb_mount_point.isEmpty())
    {
        if (usbMountPointsMap.size() == 1)
        {
            // 单个U盘，直接使用
            usb_mount_point = usbMountPointsMap.first();
            Logger::Log("RemoveImageToUsb | Using single USB from map: " + usb_mount_point.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        }
        else if (usbMountPointsMap.size() > 1)
        {
            // 多个U盘，如果 ImageSaveBaseDirectory 是U盘路径但提取失败，或者没有指定，需要用户选择
            emit wsThread->sendMessageToClient("ImageSaveErroe:USB-Multiple");
            Logger::Log("RemoveImageToUsb | Multiple USB drives detected, please specify which one to use.", LogLevel::WARNING, DeviceType::MAIN);
            return;
        }
        else
        {
            // 映射表为空，尝试使用统一的U盘挂载点获取函数（作为后备）
            if (!getUSBMountPoint(usb_mount_point))
            {
                // 获取U盘名称用于错误消息
                QString base = "/run/media/";
                QString username = QDir::home().dirName();
                QString basePath = base + username;
                QDir baseDir(basePath);
                
                if (!baseDir.exists())
                {
                    emit wsThread->sendMessageToClient("ImageSaveErroe:USB-Null");
                    Logger::Log("RemoveImageToUsb | Base directory does not exist.", LogLevel::WARNING, DeviceType::MAIN);
                }
                else
                {
                    QStringList filters;
                    filters << "*";
                    QStringList folderList = baseDir.entryList(filters, QDir::Dirs | QDir::NoDotAndDotDot);
                    folderList.removeAll("CDROM");
                    
                    if (folderList.size() == 0)
                    {
                        emit wsThread->sendMessageToClient("ImageSaveErroe:USB-Null");
                        Logger::Log("RemoveImageToUsb | No USB drive found." + basePath.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
                    }
                    else
                    {
                        emit wsThread->sendMessageToClient("ImageSaveErroe:USB-Multiple");
                        Logger::Log("RemoveImageToUsb | Multiple USB drives detected.", LogLevel::WARNING, DeviceType::MAIN);
                    }
                }
                return;
            }
        }
    }

    //const QString password = "quarcs"; // sudo 密码

    QStorageInfo storageInfo(usb_mount_point);
    if (storageInfo.isValid() && storageInfo.isReady())
    {
        if (storageInfo.isReadOnly())
        {
            // 处理1: 该路径为只读设备
            //if (!remountReadWrite(usb_mount_point, password))
            //   Logger::Log("RemoveImageToUsb | Failed to remount filesystem as read-write.", LogLevel::WARNING, DeviceType::MAIN);
            //   return;
            //}
            Logger::Log("RemoveImageToUsb | Filesystem remounted as read-write successfully.", LogLevel::INFO, DeviceType::MAIN);
        }
        Logger::Log("RemoveImageToUsb | This path is for writable devices.", LogLevel::INFO, DeviceType::MAIN);
    }
    else
    {
        Logger::Log("RemoveImageToUsb | The specified path is not a valid file system or is not ready.", LogLevel::WARNING, DeviceType::MAIN);
    }
    // 先统计需要移动的所有文件的总大小
    long long totalSize = getTotalSize(RemoveImgPath);
    if (totalSize <= 0)
    {
        Logger::Log("RemoveImageToUsb | No valid files to move or total size is 0.", LogLevel::WARNING, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("getUSBFail:No valid files to move!");
        return;
    }
    
    // 检查U盘剩余空间
    long long remaining_space = getUSBSpace(usb_mount_point);
    if (remaining_space == -1 || remaining_space <= 0)
    {
        Logger::Log("RemoveImageToUsb | USB drive has no available space or is not accessible.", LogLevel::WARNING, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("getUSBFail:USB drive has no available space!");
        return;
    }
    
    // 检查空间是否足够（总文件大小必须小于剩余空间）
    if (totalSize > remaining_space)
    {
        Logger::Log("RemoveImageToUsb | Insufficient storage space. Required: " + QString::number(totalSize).toStdString() + 
                   " bytes, Available: " + QString::number(remaining_space).toStdString() + " bytes", LogLevel::WARNING, DeviceType::MAIN);
        QString errorMsg = QString("Not enough storage space! Required: %1 MB, Available: %2 MB")
                          .arg(QString::number(totalSize / (1024.0 * 1024.0), 'f', 2))
                          .arg(QString::number(remaining_space / (1024.0 * 1024.0), 'f', 2));
        emit wsThread->sendMessageToClient("getUSBFail:" + errorMsg);
        return;
    }
    QDateTime currentDateTime = QDateTime::currentDateTime();
    QString folderName = "QUARCS_ImageSave";
    QString folderPath = usb_mount_point + "/" + folderName;

    int sumMoveImage = 0;
    for (const auto &imgPath : RemoveImgPath)
    {
        QString fileName = imgPath;
        int pos = fileName.lastIndexOf('/');
        int pos1 = fileName.lastIndexOf("/", pos - 1);
        if (pos == -1 || pos1 == -1)
        {
            Logger::Log("RemoveImageToUsb | path is error!", LogLevel::WARNING, DeviceType::MAIN);
            return;
        }
        QString destinationPath = folderPath + fileName.mid(pos1, pos - pos1 + 1);
        
        // 安全检查：避免在 /media/quarcs 路径下创建任何文件夹，避免被错误识别为U盘
        user = qEnvironmentVariable("USER");
        QString normalizedDestPath = QDir(destinationPath).absolutePath();
        if (normalizedDestPath.startsWith("/run/media/" + user ))
        {
            // 提取 /media/quarcs/ 之后的部分
            QString pathAfterMedia = normalizedDestPath.mid(14); // 去掉 "/media/quarcs/"
            
            // 检查路径格式：应该是 /media/quarcs/某个U盘名/...
            int firstSlash = pathAfterMedia.indexOf('/');
            if (firstSlash > 0)
            {
                QString usbName = pathAfterMedia.left(firstSlash);
                // 检查这个U盘名是否在映射表中（有效的U盘挂载点）
                if (!usbMountPointsMap.contains(usbName))
                {
                    QString message = QString("RemoveImageToUsb | Security check failed: Attempting to create directory in /run/media/%1. but USB name '%2' not found in mount points map. Path: %3")
                                        .arg(user)
                                        .arg(usbName)
                                        .arg(destinationPath);

                    Logger::Log(message.toStdString(), LogLevel::ERROR, DeviceType::MAIN);

                    emit wsThread->sendMessageToClient("HasMoveImgnNUmber:fail:" + QString::number(sumMoveImage));
                    continue;
                }
                // 验证路径确实在U盘挂载点下
                QString expectedMountPoint = "/run/media/" + user + "/" + usbName;
                if (!normalizedDestPath.startsWith(expectedMountPoint))
                {
                    QString message = QString("RemoveImageToUsb | Security check failed: Path does not match expected mount point. Path: %1., Expected mount point: %2")
                                        .arg(destinationPath)
                                        .arg(expectedMountPoint);

                    Logger::Log(message.toStdString(), LogLevel::ERROR, DeviceType::MAIN);

                    emit wsThread->sendMessageToClient("HasMoveImgnNUmber:fail:" + QString::number(sumMoveImage));
                    continue;
                }
            }
            else
            {
                // 路径格式不正确，可能是直接在 /media/quarcs/ 下创建文件夹
                QString message = QString("RemoveImageToUsb | Security check failed: Invalid path format in /run/media/%1. Path: %2")
                                    .arg(user)
                                    .arg(destinationPath);

                Logger::Log(message.toStdString(), LogLevel::ERROR, DeviceType::MAIN);

                emit wsThread->sendMessageToClient("HasMoveImgnNUmber:fail:" + QString::number(sumMoveImage));
                continue;
            }
        }
        // 额外检查：确保路径不是直接在 /media/quarcs 下（没有子目录）
        else if (normalizedDestPath == "/run/media/" + user)
        {
            QString message = QString("RemoveImageToUsb | Security check failed: Attempting to create directory directly at /run/media/%1. Path: %2")
                                .arg(user)
                                .arg(destinationPath);

            Logger::Log(message.toStdString(), LogLevel::ERROR, DeviceType::MAIN);

            emit wsThread->sendMessageToClient("HasMoveImgnNUmber:fail:" + QString::number(sumMoveImage));
            continue;
        }
        
        QProcess process;
//        process.start("sudo", {"-S", "mkdir", "-p", destinationPath});
        process.start("mkdir", {"-p", destinationPath});
        //if (!process.waitForStarted() || !process.write((password + "\n").toUtf8()))
        //{
        //   Logger::Log("RemoveImageToUsb | Failed to execute command: sudo mkdir.", LogLevel::WARNING, DeviceType::MAIN);
        //    emit wsThread->sendMessageToClient("HasMoveImgnNUmber:fail:" + QString::number(sumMoveImage));
        //    continue;
        //}
        process.closeWriteChannel();
        process.waitForFinished(-1);

//        process.start("sudo", {"-S", "cp", "-r", imgPath, destinationPath});
        process.start("cp", {"-r", imgPath, destinationPath});
        // if (!process.waitForStarted() || !process.write((password + "\n").toUtf8()))
        // {
        //     Logger::Log("RemoveImageToUsb | Failed to execute command: sudo cp.", LogLevel::WARNING, DeviceType::MAIN);
        //     emit wsThread->sendMessageToClient("HasMoveImgnNUmber:fail:" + QString::number(sumMoveImage));
        //     continue;
        // }
        process.closeWriteChannel();
        process.waitForFinished(-1);

        // Read the standard error output
        QByteArray stderrOutput = process.readAllStandardError();

        if (process.exitCode() == 0)
        {
            Logger::Log(QString("RemoveImageToUsb | Copied file: %1  to  %2")
            .arg(imgPath)
            .arg(destinationPath)
            .toStdString(),
            LogLevel::ERROR,
            DeviceType::MAIN);

        }
        else
        {
            Logger::Log("RemoveImageToUsb | Failed to copy file: " + imgPath.toStdString() + " to " + destinationPath.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
            // Print the error reason
            Logger::Log("RemoveImageToUsb | Error: " + QString::fromUtf8(stderrOutput).toStdString(), LogLevel::WARNING, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("HasMoveImgnNUmber:fail:" + QString::number(sumMoveImage));
            continue;
        }
        sumMoveImage++;
        emit wsThread->sendMessageToClient("HasMoveImgnNUmber:succeed:" + QString::number(sumMoveImage));
    }
}

void MainWindow::USBCheck()
{
    // AstroArch automount usb drive
    QProcess usbautomountProcess;
    usbautomountProcess.start("udiskie-mount", {"-a"});
    usbautomountProcess.closeWriteChannel();


    if (usbautomountProcess.waitForFinished()) {
        qDebug() << "USB drive installation completed successfully.";
    } else {
        qWarning() << "Error or timeout during mounting:" << usbautomountProcess.errorString();
    }
    usbautomountProcess.waitForFinished(-1);

    // 清空之前的U盘映射表
    usbMountPointsMap.clear();
    
    QString base = "/run/media/";
    QString username = QDir::home().dirName();
    QString basePath = base + username;
    QDir baseDir(basePath);

    if (!baseDir.exists())
    {
        Logger::Log("USBCheck | Base directory does not exist." + basePath.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("USBCheck:Null, Null");
        return;
    }

    // 获取所有文件夹，排除"."和".."，并且排除"CDROM"
    QStringList filters;
    filters << "*";
    QStringList folderList = baseDir.entryList(filters, QDir::Dirs | QDir::NoDotAndDotDot);
    folderList.removeAll("CDROM");

    if (folderList.size() == 0)
    {
        emit wsThread->sendMessageToClient("USBCheck:Null, Null");
        Logger::Log("USBCheck | No USB drive found." + basePath.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        return;
    }
    
    // 遍历所有U盘，验证并存储到映射表
    QStringList validUsbList;
    for (const QString &folderName : folderList)
    {
        QString usb_mount_point = basePath + folderName;
        QStorageInfo storageInfo(usb_mount_point);
        
        // 验证这是否是一个真正挂载的存储设备
        if (storageInfo.isValid() && storageInfo.isReady())
        {
            // 存储U盘信息：U盘名 -> U盘路径
            usbMountPointsMap[folderName] = usb_mount_point;
            validUsbList.append(folderName);
            
            Logger::Log("USBCheck | Found USB: " + folderName.toStdString() + " -> " + usb_mount_point.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        }
    }
    
    if (validUsbList.size() == 0)
    {
        emit wsThread->sendMessageToClient("USBCheck:Null, Null");
        Logger::Log("USBCheck | No valid USB drive found.", LogLevel::WARNING, DeviceType::MAIN);
        return;
    }
    else if (validUsbList.size() == 1)
    {
        // 单个U盘：发送U盘名和剩余空间
        QString usbName = validUsbList.at(0);
        QString usb_mount_point = usbMountPointsMap[usbName];
        long long remaining_space = getUSBSpace(usb_mount_point);
        if (remaining_space == -1)
        {
            Logger::Log("USBCheck | Check whether a USB flash drive or portable hard drive is inserted!", LogLevel::WARNING, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("USBCheck:Null, Null");
            return;
        }
        QString message = "USBCheck:" + usbName + "," + QString::number(remaining_space);
        Logger::Log("USBCheck | " + message.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient(message);
    }
    else
    {
        // 多个U盘：发送所有U盘信息
        QString message = "USBCheck:Multiple";
        QStringList usbInfoList;
        for (const QString &usbName : validUsbList)
        {
            QString usb_mount_point = usbMountPointsMap[usbName];
            long long remaining_space = getUSBSpace(usb_mount_point);
            if (remaining_space != -1)
            {
                usbInfoList.append(usbName + "," + QString::number(remaining_space));
            }
        }
        if (usbInfoList.size() > 0)
        {
            message = message + ":" + usbInfoList.join(":");
        }
        Logger::Log("USBCheck | Multiple USB drives: " + message.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient(message);
    }
}

// 获取U盘挂载点（统一函数，供其他函数复用）
bool MainWindow::getUSBMountPoint(QString &usb_mount_point)
{
    QString base = "/run/media/";
    QString username = QDir::home().dirName();
    QString basePath = base + username;
    QDir baseDir(basePath);
    
    if (!baseDir.exists())
    {
        Logger::Log("getUSBMountPoint | Base directory does not exist.", LogLevel::WARNING, DeviceType::MAIN);
        return false;
    }

    // 获取所有文件夹，排除"."和".."，并且排除"CDROM"
    QStringList filters;
    filters << "*";
    QStringList folderList = baseDir.entryList(filters, QDir::Dirs | QDir::NoDotAndDotDot);
    folderList.removeAll("CDROM");

    // 检查剩余文件夹数量是否为1
    if (folderList.size() == 1)
    {
        usb_mount_point = basePath + "/" + folderList.at(0);
        
        // 验证这是否是一个真正挂载的存储设备
        QStorageInfo storageInfo(usb_mount_point);
        if (!storageInfo.isValid() || !storageInfo.isReady())
        {
            Logger::Log("getUSBMountPoint | The directory exists but is not a valid mounted storage device.", LogLevel::WARNING, DeviceType::MAIN);
            return false;
        }
        
        Logger::Log("getUSBMountPoint | USB mount point:" + usb_mount_point.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        return true;
    }
    else if (folderList.size() == 0)
    {
        Logger::Log("getUSBMountPoint | No USB drive found." + basePath.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
        return false;
    }
    else
    {
        Logger::Log("getUSBMountPoint | Multiple USB drives detected.", LogLevel::WARNING, DeviceType::MAIN);
        return false;
    }
}

void MainWindow::GetUSBFiles(const QString &usbName, const QString &relativePath)
{
    Logger::Log("GetUSBFiles start ...", LogLevel::INFO, DeviceType::MAIN);
    
    // 必须传入U盘名
    if (usbName.isEmpty())
    {
        Logger::Log("GetUSBFiles | USB name is required.", LogLevel::WARNING, DeviceType::MAIN);
        QJsonObject errorObj;
        errorObj["error"] = "USB name is required";
        errorObj["path"] = "";
        errorObj["files"] = QJsonArray();
        QJsonDocument errorDoc(errorObj);
        emit wsThread->sendMessageToClient("USBFilesList:" + errorDoc.toJson(QJsonDocument::Compact));
        return;
    }
    
    // 根据U盘名从映射表获取挂载点路径
    if (!usbMountPointsMap.contains(usbName))
    {
        Logger::Log("GetUSBFiles | Specified USB name not found: " + usbName.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
        QJsonObject errorObj;
        errorObj["error"] = QString("USB drive not found: %1").arg(usbName);
        errorObj["path"] = "";
        errorObj["files"] = QJsonArray();
        QJsonDocument errorDoc(errorObj);
        emit wsThread->sendMessageToClient("USBFilesList:" + errorDoc.toJson(QJsonDocument::Compact));
        return;
    }
    
    QString usb_mount_point = usbMountPointsMap[usbName];
    Logger::Log("GetUSBFiles | Using USB: " + usbName.toStdString() + " -> " + usb_mount_point.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    
    // 构建完整路径
    QString fullPath = usb_mount_point;
    
    // 清理路径，防止路径遍历攻击
    QString cleanPath = relativePath;
    cleanPath.replace("..", ""); // 移除路径遍历
    cleanPath.replace("//", "/"); // 移除双斜杠
    if (cleanPath.startsWith("/"))
    {
        cleanPath = cleanPath.mid(1); // 移除开头的斜杠
    }
    if (!cleanPath.isEmpty())
    {
        fullPath = usb_mount_point + "/" + cleanPath;
    }

    // 验证目录是否存在
    QDir targetDir(fullPath);
    if (!targetDir.exists())
    {
        Logger::Log("GetUSBFiles | Target directory does not exist: " + fullPath.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
        QJsonObject errorObj;
        errorObj["error"] = "Directory not found";
        errorObj["path"] = "/" + relativePath;
        errorObj["files"] = QJsonArray();
        QJsonDocument errorDoc(errorObj);
        emit wsThread->sendMessageToClient("USBFilesList:" + errorDoc.toJson(QJsonDocument::Compact));
        return;
    }

    // 获取文件列表
    QJsonArray filesArray;
    QFileInfoList entries = targetDir.entryInfoList(QDir::AllEntries | QDir::NoDotAndDotDot, QDir::Name | QDir::DirsFirst);
    
    for (const QFileInfo &entry : entries)
    {
        QJsonObject fileObj;
        fileObj["name"] = entry.fileName();
        fileObj["isDirectory"] = entry.isDir();
        if (!entry.isDir())
        {
            fileObj["size"] = static_cast<qint64>(entry.size());
        }
        filesArray.append(fileObj);
    }

    // 构建返回结果
    QJsonObject result;
    QString displayPath = relativePath.isEmpty() ? "/" : ("/" + relativePath);
    result["path"] = displayPath;
    result["files"] = filesArray;

    QJsonDocument doc(result);
    QString jsonString = doc.toJson(QJsonDocument::Compact);
    
    Logger::Log("GetUSBFiles | Found " + QString::number(filesArray.size()).toStdString() + " items in " + fullPath.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    emit wsThread->sendMessageToClient("USBFilesList:" + jsonString);
    Logger::Log("GetUSBFiles finish!", LogLevel::INFO, DeviceType::MAIN);
}

void MainWindow::LoopSolveImage(QString Filename, int FocalLength, double CameraWidth, double CameraHeight)
{
    Logger::Log("LoopSolveImage(" + Filename.toStdString() + ") start ...", LogLevel::INFO, DeviceType::MAIN);

    if (!isLoopSolveImage)
    {
        Logger::Log("LoopSolveImage | Loop Solve Image end.", LogLevel::INFO, DeviceType::MAIN);
        return;
    }

    solveTimer.stop();
    solveTimer.disconnect();

    Tools::PlateSolve(Filename, FocalLength, CameraWidth, CameraHeight, false);

    // 启动等待赤道仪转动的定时器
    solveTimer.setSingleShot(true);

    connect(&solveTimer, &QTimer::timeout, [this, Filename]()
            {
        // 检查赤道仪状态
        if (Tools::isSolveImageFinish())
        {
            SloveResults result = Tools::ReadSolveResult(Filename, glMainCCDSizeX, glMainCCDSizeY);
            Logger::Log("LoopSolveImage | Plate Solve Result(RA_Degree, DEC_Degree):" + std::to_string(result.RA_Degree) + ", " + std::to_string(result.DEC_Degree), LogLevel::INFO, DeviceType::MAIN);

            if (result.RA_Degree == -1 && result.DEC_Degree == -1)
            {
                Logger::Log("LoopSolveImage | Solve image failed...", LogLevel::WARNING, DeviceType::MAIN);
                emit wsThread->sendMessageToClient("SolveImagefailed");
                emit wsThread->sendMessageToClient("LoopSolveImageFinished");
            }
            else
            {
                emit wsThread->sendMessageToClient("RealTimeSolveImageResult:" + QString::number(result.RA_Degree) + ":" + QString::number(result.DEC_Degree) + ":" + QString::number(Tools::RadToDegree(0)) + ":" + QString::number(Tools::RadToDegree(0)));
                emit wsThread->sendMessageToClient("LoopSolveImageFinished");
            }

            // CaptureAndSolve(glExpTime, true);
        }
        else 
        {
            solveTimer.start(1000);  // 继续等待
        } });

    solveTimer.start(1000);
}

void MainWindow::ClearSloveResultList()
{
    SloveResultList.clear();
}

void MainWindow::RecoverySloveResul()
{
    for (const auto &result : SloveResultList)
    {
        Logger::Log("RecoverySloveResul | Plate Solve Result(RA_Degree, DEC_Degree):" + std::to_string(result.RA_Degree) + ", " + std::to_string(result.DEC_Degree), LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("SolveImageResult:" + QString::number(result.RA_Degree) + ":" + QString::number(result.DEC_Degree) + ":" + QString::number(Tools::RadToDegree(0)) + ":" + QString::number(Tools::RadToDegree(0)));
        emit wsThread->sendMessageToClient("SolveFovResult:" + QString::number(result.RA_0) + ":" + QString::number(result.DEC_0) + ":" + QString::number(result.RA_1) + ":" + QString::number(result.DEC_1) + ":" + QString::number(result.RA_2) + ":" + QString::number(result.DEC_2) + ":" + QString::number(result.RA_3) + ":" + QString::number(result.DEC_3));
    }
}


// The hotspot must not be modified in AstroArch

 void MainWindow::editHotspotName(QString newName)
 {
//     Logger::Log("editHotspotName(" + newName.toStdString() + ") start ...", LogLevel::INFO, DeviceType::MAIN);
//     QString command = QString("echo 'quarcs' | sudo -S sed -i 's/^ssid=.*/ssid=%1/' /etc/NetworkManager/system-connections/RaspBerryPi-WiFi.nmconnection").arg(newName);
//
//     Logger::Log("editHotspotName | command:" + command.toStdString(), LogLevel::INFO, DeviceType::MAIN);
//
//     QProcess process;
//     process.start("bash", QStringList() << "-c" << command);
//     process.waitForFinished();
//
//     QString HostpotName = getHotspotName();
//     Logger::Log("editHotspotName | New Hotspot Name:" + HostpotName.toStdString(), LogLevel::INFO, DeviceType::MAIN);
//
//     if (HostpotName == newName)
//     {
//         emit wsThread->sendMessageToClient("EditHotspotNameSuccess");
//         // restart NetworkManager
//         process.start("sudo systemctl restart NetworkManager");
//         process.waitForFinished();
//     }
//     else
//     {
//         emit wsThread->sendMessageToClient("EditHotspotNameFailed");
//         Logger::Log("editHotspotName | Edit Hotspot name failed.", LogLevel::WARNING, DeviceType::MAIN);
//     }
 }

QString MainWindow::getHotspotName()
{
    QProcess process;

    // Force the process to use the standard C locale (English)
    QProcessEnvironment env = QProcessEnvironment::systemEnvironment();
    env.insert("LC_ALL", "C");
    process.setProcessEnvironment(env);

    // Command: nmcli -t -f active,device,ssid dev wifi
    // -t: Terse output (minimal formatting)
    // -f: Fields to display (active status, device name, and SSID)
    process.start("nmcli", QStringList() << "-t" << "-f" << "active,device,ssid" << "dev" << "wifi");

    if (!process.waitForFinished()) {
        Logger::Log("getHotspotName | Error: Process failed to finish", LogLevel::WARNING, DeviceType::MAIN);
        return "N/A";
    }

    QString output = process.readAllStandardOutput();
    Logger::Log("getHotspotName | Raw output: " + output.toStdString(), LogLevel::INFO, DeviceType::MAIN);

    // Split output into lines
    QStringList lines = output.split("\n", Qt::SkipEmptyParts);

    for (const QString& line : lines) {
        // Since LC_ALL=C is set, we only need to check for "yes"
        if (line.startsWith("yes")) {
            // Line format is "active:device:ssid" (e.g., "yes:wlan0:MyRaspberryHotspot")
            QStringList fields = line.split(":");
            if (fields.size() >= 3) {
                QString ssid = fields.at(2).trimmed();
                if (!ssid.isEmpty()) {
                    return ssid;
                }
            }
        }
    }

    // Return default value if no active connection or SSID found
    Logger::Log("getHotspotName | No active SSID found", LogLevel::WARNING, DeviceType::MAIN);
    return "N/A";
}

/**
 * @brief 关闭当前热点指定秒数后再重新启动
 *        实现方式：使用 nmcli 将指定连接 down，然后通过 QTimer 在 delaySeconds 秒后 up。
 *        注意：该操作会在一段时间内中断当前 WiFi 热点和网络连接。
 */

// The hotspot must not be restarted in AstroArch

 void MainWindow::restartHotspotWithDelay(int delaySeconds)
 {
//     Logger::Log("restartHotspotWithDelay(" + std::to_string(delaySeconds) + ") start ...",
//                 LogLevel::INFO, DeviceType::MAIN);
//
//     // 当前热点连接名称（与 getHotspotName 读取的配置文件一致）
//     const QString connectionName = "RaspBerryPi-WiFi";
//
//     // 先关闭当前热点
//     {
//         QString command = QString("echo 'quarcs' | sudo -S nmcli connection down '%1'").arg(connectionName);
//         Logger::Log("restartHotspotWithDelay | down command:" + command.toStdString(),
//                     LogLevel::INFO, DeviceType::MAIN);
//
//         QProcess process;
//         process.start("bash", QStringList() << "-c" << command);
//         process.waitForFinished();
//
//         QString output = process.readAllStandardOutput();
//         QString errorOutput = process.readAllStandardError();
//         Logger::Log("restartHotspotWithDelay | down output:" + output.toStdString(),
//                     LogLevel::INFO, DeviceType::MAIN);
//         if (!errorOutput.isEmpty())
//         {
//             Logger::Log("restartHotspotWithDelay | down error:" + errorOutput.toStdString(),
//                         LogLevel::WARNING, DeviceType::MAIN);
//         }
//     }
//
//     // 使用 QTimer 在 delaySeconds 秒后重新启动热点，避免阻塞主线程
//     int delayMs = std::max(0, delaySeconds) * 1000;
//
//     QTimer::singleShot(delayMs, this, [this, connectionName]() {
//         Logger::Log("restartHotspotWithDelay | starting hotspot again ...",
//                     LogLevel::INFO, DeviceType::MAIN);
//
//         QString command = QString("echo 'quarcs' | sudo -S nmcli connection up '%1'").arg(connectionName);
//         Logger::Log("restartHotspotWithDelay | up command:" + command.toStdString(),
//                     LogLevel::INFO, DeviceType::MAIN);
//
//         QProcess process;
//         process.start("bash", QStringList() << "-c" << command);
//         process.waitForFinished();
//
//         QString output = process.readAllStandardOutput();
//         QString errorOutput = process.readAllStandardError();
//         Logger::Log("restartHotspotWithDelay | up output:" + output.toStdString(),
//                     LogLevel::INFO, DeviceType::MAIN);
//         if (!errorOutput.isEmpty())
//         {
//             Logger::Log("restartHotspotWithDelay | up error:" + errorOutput.toStdString(),
//                         LogLevel::WARNING, DeviceType::MAIN);
//         }
//
//         Logger::Log("restartHotspotWithDelay | hotspot restart sequence finished",
//                     LogLevel::INFO, DeviceType::MAIN);
//     });
 }

void MainWindow::SendDebugToVueClient(const QString &msg)
{
    emit wsThread->sendMessageToClient("SendDebugMessage|" + msg);
}

void MainWindow::customMessageHandler(QtMsgType type, const QMessageLogContext &context, const QString &msg)
{
    Q_UNUSED(context);

    if (!instance)
    {
        return;
    }

    QString currentTime = QDateTime::currentDateTime().toString("yyyy-MM-dd hh:mm:ss");
    QString typeStr;
    switch (type)
    {
    case QtDebugMsg:
        typeStr = "Debug";
        break;
    case QtInfoMsg:
        typeStr = "Info";
        break;
    case QtWarningMsg:
        typeStr = "Warning";
        break;
    case QtCriticalMsg:
        typeStr = "Critical";
        break;
    default:
        typeStr = "Unknown";
    }

    // 统一格式化日志消息
    QString logMessage = QString("[Server] %1 | %2 | %3").arg(currentTime).arg(typeStr).arg(msg);

    // // 将日志消息添加到缓存
    // instance->logCache.push_back(logMessage);

    // 统一输出到标准错误，无论日志类型
    fprintf(stderr, "%s\n", logMessage.toLocal8Bit().constData());

    // 发送到客户端的Vue应用，包括日志类型和消息
    instance->SendDebugToVueClient(typeStr + "|" + msg);
}
void MainWindow::ConnectDriver(QString DriverName, QString DriverType)
{
    Logger::Log("ConnectDriver(" + DriverName.toStdString() + ", " + DriverType.toStdString() + ") start ...", LogLevel::INFO, DeviceType::MAIN);
    if (DriverName == "" || DriverType == "")
    {
        Logger::Log("ConnectDriver | DriverName(" + DriverName.toStdString() + ") or DriverType(" + DriverType.toStdString() + ") is Null", LogLevel::WARNING, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConnectFailed:DriverName or DriverType is Null.");
        emit wsThread->sendMessageToClient("ConnectDriverFailed:DriverName or DriverType is Null.");
        return;
    }

    int driverCode = -1;
    bool isDriverConnected = false;
    for (int i = 0; i < systemdevicelist.system_devices.size(); i++)
    {
        if (systemdevicelist.system_devices[i].Description == DriverType)
        {
            if (systemdevicelist.system_devices[i].isConnect)
            {
                Logger::Log("ConnectDriver | Driver(" + DriverName.toStdString() + ") is already connected", LogLevel::INFO, DeviceType::MAIN);
            }
            else
            {
                if (ConnectDriverList.contains(DriverName))
                {
                    Logger::Log("ConnectDriver | Driver(" + DriverName.toStdString() + ") is already in ConnectDriverList", LogLevel::INFO, DeviceType::MAIN);
                }
                else
                {
                    Logger::Log("ConnectDriver | Driver(" + DriverName.toStdString() + ") is not connected, start to connect", LogLevel::INFO, DeviceType::MAIN);
                    ConnectDriverList.push_back(DriverName);
                    Tools::startIndiDriver(DriverName);
                }
            }
            driverCode = i;
            isDriverConnected = true;
            break;
        }
    }

    if (!isDriverConnected)
    {
        Logger::Log("ConnectDriver | " + DriverType.toStdString() + " Driver(" + DriverName.toStdString() + ") is not selected, start to connect", LogLevel::INFO, DeviceType::MAIN);
        if (DriverType == "Mount")
        {
            driverCode = 0;
            systemdevicelist.system_devices[0].Description = "Mount";
            systemdevicelist.system_devices[0].DriverIndiName = DriverName;
        }
        else if (DriverType == "Guider")
        {
            driverCode = 1;
            systemdevicelist.system_devices[1].Description = "Guider";
            systemdevicelist.system_devices[1].DriverIndiName = DriverName;
        }
        else if (DriverType == "PoleCamera")
        {
            driverCode = 2;
            systemdevicelist.system_devices[2].Description = "PoleCamera";
            systemdevicelist.system_devices[2].DriverIndiName = DriverName;
        }
        else if (DriverType == "MainCamera")
        {
            driverCode = 20;
            systemdevicelist.system_devices[20].Description = "MainCamera";
            systemdevicelist.system_devices[20].DriverIndiName = DriverName;
        }
        else if (DriverType == "CFW")
        {
            driverCode = 21;
            systemdevicelist.system_devices[21].Description = "CFW";
            systemdevicelist.system_devices[21].DriverIndiName = DriverName;
        }
        else if (DriverType == "Focuser")
        {
            driverCode = 22;
            systemdevicelist.system_devices[22].Description = "Focuser";
            systemdevicelist.system_devices[22].DriverIndiName = DriverName;
        }
        else
        {
            Logger::Log("ConnectDriver | DriverType(" + DriverType.toStdString() + ") is not supported.", LogLevel::WARNING, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("ConnectFailed:DriverType is not supported.");
            emit wsThread->sendMessageToClient("ConnectDriverFailed:DriverType is not supported.");
            Tools::stopIndiDriver(DriverName);
            ConnectDriverList.removeAll(DriverName);
            return;
        }
        Tools::startIndiDriver(DriverName);
        ConnectDriverList.push_back(DriverName);
    }
    sleep(1);
    if (indi_Client->isServerConnected() == false)
    {
        Logger::Log("ConnectDriver | indi Client is not connected, try to connect", LogLevel::INFO, DeviceType::MAIN);
        connectIndiServer(indi_Client);
        if (indi_Client->isServerConnected() == false)
        {
            Logger::Log("ConnectDriver | Connect indi server failed", LogLevel::WARNING, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("ConnectFailed:Connect indi server failed.");
            emit wsThread->sendMessageToClient("ConnectDriverFailed:Connect indi server failed.");
            Tools::stopIndiDriver(DriverName);
            ConnectDriverList.removeAll(DriverName);
            return;
        }
    }
    sleep(1);
    int time = 0;
    while (time < 10)
    {
        if (indi_Client->GetDeviceCount() > 0)
        {
            Logger::Log("ConnectDriver | Find device!", LogLevel::INFO, DeviceType::MAIN);
            break;
        }
        Logger::Log("ConnectDriver | Wait find device...", LogLevel::INFO, DeviceType::MAIN);
        QThread::msleep(1000);
        time++;
    }
    sleep(1);
    if (indi_Client->GetDeviceCount() == 0)
    {
        Logger::Log("ConnectDriver | No device found", LogLevel::WARNING, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("ConnectDriverFailed:No device found.");
        Tools::stopIndiDriver(DriverName);
        ConnectDriverList.removeAll(DriverName);
        return;
    }
    // 记录连接的设备的id列表
    std::vector<int> connectedDeviceIdList;
    for (int i = 0; i < indi_Client->GetDeviceCount(); i++)
    {

        if (indi_Client->GetDeviceFromList(i)->getDriverExec() == DriverName)
        {
            if (indi_Client->GetDeviceFromList(i)->isConnected())
            {
                Logger::Log("ConnectDriver | Device(" + std::string(indi_Client->GetDeviceFromList(i)->getDeviceName()) + ") is connected", LogLevel::INFO, DeviceType::MAIN);
                bool isDeviceBind = false;
                for (int j = 0; j < systemdevicelist.system_devices.size(); j++)
                {
                    if (systemdevicelist.system_devices[j].DeviceIndiName == indi_Client->GetDeviceNameFromList(i).c_str())
                    {
                        isDeviceBind = true;
                    }
                }
                if (!isDeviceBind)
                {
                    Logger::Log("ConnectDriver | Device(" + std::string(indi_Client->GetDeviceFromList(i)->getDeviceName()) + ") is not bind, start to disconnect", LogLevel::INFO, DeviceType::MAIN);
                    indi_Client->disconnectDevice(indi_Client->GetDeviceFromList(i)->getDeviceName());
                    time = 0;
                    while (indi_Client->GetDeviceFromList(i)->isConnected() && time < 30)
                    {
                        Logger::Log("ConnectDriver | Wait for disconnect" + std::string(indi_Client->GetDeviceNameFromList(i).c_str()), LogLevel::INFO, DeviceType::MAIN);
                        QThread::msleep(1000);
                        time++;
                    }
                    if (!indi_Client->GetDeviceFromList(i)->isConnected())
                    {
                        Logger::Log("ConnectDriver | Device(" + std::string(indi_Client->GetDeviceFromList(i)->getDeviceName()) + ") is disconnected, start to connect", LogLevel::INFO, DeviceType::MAIN);
                    }
                    else
                    {
                        Logger::Log("ConnectDriver | Device(" + std::string(indi_Client->GetDeviceFromList(i)->getDeviceName()) + ") is not disconnected", LogLevel::WARNING, DeviceType::MAIN);
                    }

                    time = 0;
                    // 根据 DriverName 和 DriverType 在 systemdevicelist 中找到对应的设备下标，用于获取正确的波特率
                    int sysIndex = -1;
                    for (int idx = 0; idx < systemdevicelist.system_devices.size(); idx++)
                    {
                        if (systemdevicelist.system_devices[idx].DriverIndiName == DriverName &&
                            systemdevicelist.system_devices[idx].Description == DriverType)
                        {
                            sysIndex = idx;
                            break;
                        }
                    }

                    // 在正式连接前，仅在用户手动选择串口时应用覆盖设置；默认模式不改端口
                    if (DriverType == "Focuser" && !focuserSerialPortOverride.isEmpty())
                    {
                        indi_Client->setDevicePort(indi_Client->GetDeviceFromList(i), focuserSerialPortOverride);
                        Logger::Log("ConnectDriver | Focuser initial Port set to: " + focuserSerialPortOverride.toStdString(),
                                    LogLevel::INFO, DeviceType::MAIN);
                    }
                    else if (DriverType == "Mount" && !mountSerialPortOverride.isEmpty())
                    {
                        indi_Client->setDevicePort(indi_Client->GetDeviceFromList(i), mountSerialPortOverride);
                        Logger::Log("ConnectDriver | Mount initial Port set to: " + mountSerialPortOverride.toStdString(),
                                    LogLevel::INFO, DeviceType::MAIN);
                    }

                    int baudRateToUse = 9600;
                    if (sysIndex >= 0)
                    {
                        baudRateToUse = systemdevicelist.system_devices[sysIndex].BaudRate;
                    }
                    else
                    {
                        Logger::Log("ConnectDriver | Unable to find SystemDevice for Driver(" + DriverName.toStdString() + "), use default baud 9600",
                                    LogLevel::WARNING, DeviceType::MAIN);
                    }
                    indi_Client->setBaudRate(indi_Client->GetDeviceFromList(i), baudRateToUse);
                    indi_Client->connectDevice(indi_Client->GetDeviceFromList(i)->getDeviceName());
                    while (!indi_Client->GetDeviceFromList(i)->isConnected() && time < 15)
                    {
                        Logger::Log("ConnectDriver | Wait for connect" + std::string(indi_Client->GetDeviceFromList(i)->getDeviceName()) + ",state:" + std::to_string(indi_Client->GetDeviceFromList(i)->isConnected()), LogLevel::INFO, DeviceType::MAIN);
                        QThread::msleep(1000);
                        time++;
                    }
                    if (!indi_Client->GetDeviceFromList(i)->isConnected())
                    {
                        Logger::Log("ConnectDriver | Device (" + std::string(indi_Client->GetDeviceNameFromList(i).c_str()) + ") is not exist", LogLevel::WARNING, DeviceType::MAIN);
                        emit wsThread->sendMessageToClient("deleteDeviceAllocationList:" + QString::fromUtf8(indi_Client->GetDeviceNameFromList(i).c_str()));
                        indi_Client->RemoveDevice(indi_Client->GetDeviceNameFromList(i).c_str());
                        for (int j = 0; j < connectedDeviceIdList.size(); j++)
                        {
                            if (connectedDeviceIdList[j] == i)
                            {
                                connectedDeviceIdList.erase(connectedDeviceIdList.begin() + j);
                                break;
                            }
                        }
                    }
                    else
                    {
                        connectedDeviceIdList.push_back(i);
                    }
                }
                else
                {
                    Logger::Log("ConnectDriver | Device(" + std::string(indi_Client->GetDeviceFromList(i)->getDeviceName()) + ") is bind, is Used", LogLevel::INFO, DeviceType::MAIN);
                }
            }
            else
            {
                Logger::Log("ConnectDriver | Device(" + std::string(indi_Client->GetDeviceFromList(i)->getDeviceName()) + ") is connecting...", LogLevel::INFO, DeviceType::MAIN);

                // 根据 DriverName 和 DriverType 在 systemdevicelist 中找到对应的设备下标，用于获取正确的波特率
                int sysIndex = -1;
                for (int idx = 0; idx < systemdevicelist.system_devices.size(); idx++)
                {
                    if (systemdevicelist.system_devices[idx].DriverIndiName == DriverName &&
                        systemdevicelist.system_devices[idx].Description == DriverType)
                    {
                        sysIndex = idx;
                        break;
                    }
                }
                // 在正式连接前，仅在用户手动选择串口时应用覆盖设置；默认模式不改端口
                if (DriverType == "Focuser" && !focuserSerialPortOverride.isEmpty())
                {
                    indi_Client->setDevicePort(indi_Client->GetDeviceFromList(i), focuserSerialPortOverride);
                    Logger::Log("ConnectDriver | Focuser initial Port set to: " + focuserSerialPortOverride.toStdString(),
                                LogLevel::INFO, DeviceType::MAIN);
                }
                else if (DriverType == "Mount" && !mountSerialPortOverride.isEmpty())
                {
                    indi_Client->setDevicePort(indi_Client->GetDeviceFromList(i), mountSerialPortOverride);
                    Logger::Log("ConnectDriver | Mount initial Port set to: " + mountSerialPortOverride.toStdString(),
                                LogLevel::INFO, DeviceType::MAIN);
                }

                int baudRateToUse = 9600;
                if (sysIndex >= 0)
                {
                    baudRateToUse = systemdevicelist.system_devices[sysIndex].BaudRate;
                }
                else
                {
                    Logger::Log("ConnectDriver | Unable to find SystemDevice for Driver(" + DriverName.toStdString() + "), use default baud 9600",
                                LogLevel::WARNING, DeviceType::MAIN);
                }
                indi_Client->setBaudRate(indi_Client->GetDeviceFromList(i), baudRateToUse);
                indi_Client->connectDevice(indi_Client->GetDeviceNameFromList(i).c_str());
                int waitTime = 0;
                bool connectState = false;
                while (waitTime < 15)
                {
                    Logger::Log("ConnectDriver | Wait for Connect " + std::string(indi_Client->GetDeviceNameFromList(i).c_str()), LogLevel::INFO, DeviceType::MAIN);
                    QThread::msleep(1000); // 等待1秒
                    waitTime++;
                    if (indi_Client->GetDeviceFromList(i)->isConnected())
                    {
                        connectState = true;
                        break;
                    }
                }
                if (connectState)
                {
                    connectedDeviceIdList.push_back(i);
                }else{
                    Logger::Log("ConnectDriver | Device (" + std::string(indi_Client->GetDeviceFromList(i)->getDeviceName()) + ") is not connected,try to update port", LogLevel::WARNING, DeviceType::MAIN);
                    // 特殊处理电调和赤道仪的连接
                    if (DriverType == "Focuser")
                    {
                        // 电调当前的串口
                        QString DevicePort;
                        indi_Client->getDevicePort(indi_Client->GetDeviceFromList(i), DevicePort);
                        if (detector.detectDeviceTypeForPort(DevicePort) != "Focuser")
                        {
                            // 识别到当前设备的串口不是电调的串口,需更新
                            // 正确的串口是detector.getFocuserPort()
                            QString realFocuserPort = detector.getFocuserPort();
                            if (!realFocuserPort.isEmpty())
                            {
                                indi_Client->setDevicePort(indi_Client->GetDeviceFromList(i), realFocuserPort);
                                // 将自动匹配到的端口同步到覆盖值，保证后续连接与前端显示一致
                                focuserSerialPortOverride = realFocuserPort;
                                Logger::Log("ConnectDriver | Focuser Device (" + std::string(indi_Client->GetDeviceFromList(i)->getDeviceName()) + ") Port is updated to: " + realFocuserPort.toStdString(), LogLevel::INFO, DeviceType::MAIN);
                            }
                            else
                            {
                                Logger::Log("No matched Focuser port found by detector.", LogLevel::WARNING, DeviceType::MAIN);
                                continue;
                            }
                        }else{
                            Logger::Log("ConnectDriver | Focuser Device (" + std::string(indi_Client->GetDeviceFromList(i)->getDeviceName()) + ") Port is correct.", LogLevel::INFO, DeviceType::MAIN);
                        }
                    }
                    else if (DriverType == "Mount")
                    {
                        // 赤道仪当前的串口
                        QString DevicePort;
                        indi_Client->getDevicePort(indi_Client->GetDeviceFromList(i), DevicePort);
                        if (detector.detectDeviceTypeForPort(DevicePort) != "Mount")
                        {
                            // 识别到当前设备的串口不是赤道仪的串口,需更新
                            // 正确的串口是detector.getMountPort()
                            QString realMountPort = detector.getMountPort();
                            if (!realMountPort.isEmpty())
                            {
                                indi_Client->setDevicePort(indi_Client->GetDeviceFromList(i), realMountPort);
                                // 将自动匹配到的端口同步到覆盖值，保证后续连接与前端显示一致
                                mountSerialPortOverride = realMountPort;
                                Logger::Log("ConnectDriver | Mount Device (" + std::string(indi_Client->GetDeviceFromList(i)->getDeviceName()) + ") Port is updated to: " + realMountPort.toStdString(), LogLevel::INFO, DeviceType::MAIN);
                            }
                            else
                            {
                                Logger::Log("No matched Mount port found by detector.", LogLevel::WARNING, DeviceType::MAIN);
                                continue;
                            }
                        }else{
                            Logger::Log("ConnectDriver | Mount Device (" + std::string(indi_Client->GetDeviceFromList(i)->getDeviceName()) + ") Port is correct.", LogLevel::INFO, DeviceType::MAIN);
                        }
                    }else{
                        Logger::Log("ConnectDriver | Device (" + std::string(indi_Client->GetDeviceFromList(i)->getDeviceName()) + ") Port is not updated.", LogLevel::WARNING, DeviceType::MAIN);
                        emit wsThread->sendMessageToClient("deleteDeviceAllocationList:" + QString::fromUtf8(indi_Client->GetDeviceNameFromList(i).c_str()));
                        indi_Client->disconnectDevice(indi_Client->GetDeviceNameFromList(i).c_str());
                        Logger::Log("ConnectDriver | Device (" + std::string(indi_Client->GetDeviceNameFromList(i).c_str()) + ") is not exist", LogLevel::WARNING, DeviceType::MAIN);
                        indi_Client->RemoveDevice(indi_Client->GetDeviceNameFromList(i).c_str());
                        continue;
                    }
                    // 根据 DriverName 和 DriverType 在 systemdevicelist 中找到对应的设备下标，用于获取正确的波特率
                    int sysIndex = -1;
                    for (int idx = 0; idx < systemdevicelist.system_devices.size(); idx++)
                    {
                        if (systemdevicelist.system_devices[idx].DriverIndiName == DriverName &&
                            systemdevicelist.system_devices[idx].Description == DriverType)
                        {
                            sysIndex = idx;
                            break;
                        }
                    }
                    int baudRateToUse = 9600;
                    if (sysIndex >= 0)
                    {
                        baudRateToUse = systemdevicelist.system_devices[sysIndex].BaudRate;
                    }
                    else
                    {
                        Logger::Log("ConnectDriver | Unable to find SystemDevice for Driver(" + DriverName.toStdString() + "), use default baud 9600",
                                    LogLevel::WARNING, DeviceType::MAIN);
                    }
                    indi_Client->setBaudRate(indi_Client->GetDeviceFromList(i), baudRateToUse);
                    indi_Client->connectDevice(indi_Client->GetDeviceNameFromList(i).c_str());
                    int waitTime = 0;
                    bool connectState = false;
                    while (waitTime < 15)
                    {
                        Logger::Log("ConnectDriver | Wait for Connect " + std::string(indi_Client->GetDeviceNameFromList(i).c_str()), LogLevel::INFO, DeviceType::MAIN);
                        QThread::msleep(1000); // 等待1秒
                        waitTime++;
                        if (indi_Client->GetDeviceFromList(i)->isConnected())
                        {
                            connectState = true;
                            break;
                        }
                    }
                    if (connectState)
                    {
                        connectedDeviceIdList.push_back(i);
                    }
                    else
                    {
                        Logger::Log("ConnectDriver | Device (" + std::string(indi_Client->GetDeviceNameFromList(i).c_str()) + ") is not connected,try to update port", LogLevel::WARNING, DeviceType::MAIN);

                        // 若为 Mount/Focuser 串口设备，连接失败时提示前端弹出串口选择界面
                        if (DriverType == "Mount" || DriverType == "Focuser")
                        {
                            sendSerialPortOptions(DriverType);
                            emit wsThread->sendMessageToClient("RequestSerialPortSelection:" + DriverType);
                        }

                        emit wsThread->sendMessageToClient("deleteDeviceAllocationList:" + QString::fromUtf8(indi_Client->GetDeviceNameFromList(i).c_str()));
                        indi_Client->disconnectDevice(indi_Client->GetDeviceNameFromList(i).c_str());
                        Logger::Log("ConnectDriver | Device (" + std::string(indi_Client->GetDeviceNameFromList(i).c_str()) + ") is not exist", LogLevel::WARNING, DeviceType::MAIN);
                        indi_Client->RemoveDevice(indi_Client->GetDeviceNameFromList(i).c_str());
                        continue;
                    }
                    // emit wsThread->sendMessageToClient("deleteDeviceAllocationList:" + QString::fromUtf8(indi_Client->GetDeviceNameFromList(i).c_str()));
                    // indi_Client->disconnectDevice(indi_Client->GetDeviceNameFromList(i).c_str());
                    // Logger::Log("ConnectDriver | Device (" + std::string(indi_Client->GetDeviceNameFromList(i).c_str()) + ") is not exist", LogLevel::WARNING, DeviceType::MAIN);
                    // indi_Client->RemoveDevice(indi_Client->GetDeviceNameFromList(i).c_str());
                }   
            }
        }
    }

    if (connectedDeviceIdList.size() == 0)
    {
        Logger::Log("ConnectDriver | Driver(" + DriverName.toStdString() + ") No Device found.", LogLevel::WARNING, DeviceType::MAIN);
        Tools::stopIndiDriver(DriverName);
        int index = ConnectDriverList.indexOf(DriverName);
        if (index != -1)
        {                                      // 如果找到了
            ConnectDriverList.removeAt(index); // 从列表中删除
            Logger::Log("Driver removed successfully: " + DriverName.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        }
        else
        {
            Logger::Log("Driver not found in list: " + DriverName.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
        }
        emit wsThread->sendMessageToClient("ConnectDriverFailed:Driver(" + DriverName + ") No Device found.");
        return;
    }

    ConnectedCCDList.clear();
    ConnectedTELESCOPEList.clear();
    ConnectedFOCUSERList.clear();
    ConnectedFILTERList.clear();

    // 判断连接设备的类型
    for (int i = 0; i < connectedDeviceIdList.size(); i++)
    {
        if (indi_Client->GetDeviceFromList(connectedDeviceIdList[i])->isConnected())
        {
            if (indi_Client->GetDeviceFromList(connectedDeviceIdList[i])->getDriverInterface() & INDI::BaseDevice::CCD_INTERFACE)
            {
                Logger::Log("ConnectDriver | We received a CCD!", LogLevel::INFO, DeviceType::MAIN);
                ConnectedCCDList.push_back(connectedDeviceIdList[i]);
            }
            else if (indi_Client->GetDeviceFromList(connectedDeviceIdList[i])->getDriverInterface() & INDI::BaseDevice::FILTER_INTERFACE)
            {
                Logger::Log("ConnectDriver | We received a FILTER!", LogLevel::INFO, DeviceType::MAIN);
                ConnectedFILTERList.push_back(connectedDeviceIdList[i]);
            }
            else if (indi_Client->GetDeviceFromList(connectedDeviceIdList[i])->getDriverInterface() & INDI::BaseDevice::TELESCOPE_INTERFACE)
            {
                Logger::Log("ConnectDriver | We received a TELESCOPE!", LogLevel::INFO, DeviceType::MAIN);
                ConnectedTELESCOPEList.push_back(connectedDeviceIdList[i]);
            }
            else if (indi_Client->GetDeviceFromList(connectedDeviceIdList[i])->getDriverInterface() & INDI::BaseDevice::FOCUSER_INTERFACE)
            {
                Logger::Log("ConnectDriver | We received a FOCUSER!", LogLevel::INFO, DeviceType::MAIN);
                ConnectedFOCUSERList.push_back(connectedDeviceIdList[i]);
            }
        }
        else
        {
            Logger::Log("ConnectDriver | Connect failed device:" + std::string(indi_Client->GetDeviceFromList(connectedDeviceIdList[i])->getDeviceName()), LogLevel::WARNING, DeviceType::MAIN);
        }
    }

    QStringList SelectedCameras;
    for (int i = 0; i < systemdevicelist.system_devices.size(); i++)
    {
        if (systemdevicelist.system_devices[i].DriverIndiName == DriverName)
        {
            SelectedCameras.push_back(systemdevicelist.system_devices[i].Description);
        }
    }
    Logger::Log("ConnectDriver | Driver(" + DriverName.toStdString() + ") supports " + std::to_string(SelectedCameras.size()) + " devices", LogLevel::INFO, DeviceType::MAIN);
    for (auto Camera : SelectedCameras)
    {
        Logger::Log("ConnectDriver | Driver(" + DriverName.toStdString() + ") supports " + Camera.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    }

    Logger::Log("ConnectDriver | Number of Connected CCD:" + std::to_string(ConnectedCCDList.size()), LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("ConnectDriver | Number of Connected TELESCOPE:" + std::to_string(ConnectedTELESCOPEList.size()), LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("ConnectDriver | Number of Connected FOCUSER:" + std::to_string(ConnectedFOCUSERList.size()), LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("ConnectDriver | Number of Connected FILTER:" + std::to_string(ConnectedFILTERList.size()), LogLevel::INFO, DeviceType::MAIN);

    // 判断连接设备的数量,
    bool EachDeviceOne = true;

    if (SelectedCameras.size() == 1 && ConnectedCCDList.size() == 1)
    {
        Logger::Log("ConnectDriver | The Camera Selected and Connected are Both 1", LogLevel::INFO, DeviceType::MAIN);
        // 修复：检查向量和索引有效性
        if (SelectedCameras.empty() || ConnectedCCDList.empty()) {
            Logger::Log("ConnectDriver | SelectedCameras or ConnectedCCDList is empty", LogLevel::ERROR, DeviceType::MAIN);
        } else if (ConnectedCCDList[0] >= 0 && ConnectedCCDList[0] < indi_Client->GetDeviceCount()) {
            INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedCCDList[0]);
            if (device == nullptr) {
                Logger::Log("ConnectDriver | GetDeviceFromList returned nullptr for ConnectedCCDList[0]", LogLevel::ERROR, DeviceType::MAIN);
            } else if (SelectedCameras[0] == "Guider")
            {
                dpGuider = device;
                if (systemdevicelist.system_devices.size() > 1) {
                    systemdevicelist.system_devices[1].isConnect = true;
                }
                indi_Client->disconnectDevice(device->getDeviceName());
                sleep(1);
                call_phd_whichCamera(device->getDeviceName());
                // PHD2 connect status
                AfterDeviceConnect(dpGuider);
            }
            else if (SelectedCameras[0] == "PoleCamera")
            {
                dpPoleScope = device;
                if (systemdevicelist.system_devices.size() > 2) {
                    systemdevicelist.system_devices[2].isConnect = true;
                }
                AfterDeviceConnect(dpPoleScope);
            }
            else if (SelectedCameras[0] == "MainCamera")
            {
                Logger::Log("ConnectDriver | MainCamera Connected Success!", LogLevel::INFO, DeviceType::MAIN);
                dpMainCamera = device;
                if (systemdevicelist.system_devices.size() > 20) {
                    systemdevicelist.system_devices[20].isConnect = true;
                }
                AfterDeviceConnect(dpMainCamera);
            }
        }
    }
    else if (SelectedCameras.size() > 1 || ConnectedCCDList.size() > 1)
    {
        EachDeviceOne = false;
        for (int i = 0; i < ConnectedCCDList.size(); i++)
        {
            // 修复：检查索引有效性
            if (ConnectedCCDList[i] >= 0 && ConnectedCCDList[i] < indi_Client->GetDeviceCount()) {
                INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedCCDList[i]);
                if (device != nullptr) {
                    emit wsThread->sendMessageToClient("DeviceToBeAllocated:CCD:" + QString::number(ConnectedCCDList[i]) + ":" + QString::fromUtf8(device->getDeviceName())); // already allocated
                }
            }
        }
    }

    if (ConnectedTELESCOPEList.size() == 1)
    {
        Logger::Log("ConnectDriver | Mount Connected Success!", LogLevel::INFO, DeviceType::MAIN);
        // 修复：检查向量和索引有效性
        if (!ConnectedTELESCOPEList.empty() && ConnectedTELESCOPEList[0] >= 0 && ConnectedTELESCOPEList[0] < indi_Client->GetDeviceCount()) {
            INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedTELESCOPEList[0]);
            if (device != nullptr) {
                dpMount = device;
                if (systemdevicelist.system_devices.size() > 0) {
                    systemdevicelist.system_devices[0].isConnect = true;
                }
                AfterDeviceConnect(dpMount);
            }
        }
    }
    else if (ConnectedTELESCOPEList.size() > 1)
    {
        EachDeviceOne = false;
        for (int i = 0; i < ConnectedTELESCOPEList.size(); i++)
        {
            // 修复：检查索引有效性
            if (ConnectedTELESCOPEList[i] >= 0 && ConnectedTELESCOPEList[i] < indi_Client->GetDeviceCount()) {
                INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedTELESCOPEList[i]);
                if (device != nullptr) {
                    emit wsThread->sendMessageToClient("DeviceToBeAllocated:Mount:" + QString::number(ConnectedTELESCOPEList[i]) + ":" + QString::fromUtf8(device->getDeviceName()));
                }
            }
        }
    }

    if (ConnectedFOCUSERList.size() == 1)
    {
        Logger::Log("ConnectDriver | Focuser Connected Success!", LogLevel::INFO, DeviceType::MAIN);
        // 修复：检查向量和索引有效性
        if (!ConnectedFOCUSERList.empty() && ConnectedFOCUSERList[0] >= 0 && ConnectedFOCUSERList[0] < indi_Client->GetDeviceCount()) {
            INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedFOCUSERList[0]);
            if (device != nullptr) {
                dpFocuser = device;
                if (systemdevicelist.system_devices.size() > 22) {
                    systemdevicelist.system_devices[22].isConnect = true;
                }
                AfterDeviceConnect(dpFocuser);
            }
        }
    }
    else if (ConnectedFOCUSERList.size() > 1)
    {
        EachDeviceOne = false;
        for (int i = 0; i < ConnectedFOCUSERList.size(); i++)
        {
            // 修复：检查索引有效性
            if (ConnectedFOCUSERList[i] >= 0 && ConnectedFOCUSERList[i] < indi_Client->GetDeviceCount()) {
                INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedFOCUSERList[i]);
                if (device != nullptr) {
                    emit wsThread->sendMessageToClient("DeviceToBeAllocated:Focuser:" + QString::number(ConnectedFOCUSERList[i]) + ":" + QString::fromUtf8(device->getDeviceName()));
                }
            }
        }
    }

    if (ConnectedFILTERList.size() == 1)
    {
        Logger::Log("ConnectDriver | Filter Connected Success!", LogLevel::INFO, DeviceType::MAIN);
        // 修复：检查向量和索引有效性
        if (!ConnectedFILTERList.empty() && ConnectedFILTERList[0] >= 0 && ConnectedFILTERList[0] < indi_Client->GetDeviceCount()) {
            INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedFILTERList[0]);
            if (device != nullptr) {
                dpCFW = device;
                if (systemdevicelist.system_devices.size() > 21) {
                    systemdevicelist.system_devices[21].isConnect = true;
                }
                AfterDeviceConnect(dpCFW);
            }
        }
    }
    else if (ConnectedFILTERList.size() > 1)
    {
        EachDeviceOne = false;
        for (int i = 0; i < ConnectedFILTERList.size(); i++)
        {
            // 修复：检查索引有效性
            if (ConnectedFILTERList[i] >= 0 && ConnectedFILTERList[i] < indi_Client->GetDeviceCount()) {
                INDI::BaseDevice *device = indi_Client->GetDeviceFromList(ConnectedFILTERList[i]);
                if (device != nullptr) {
                    emit wsThread->sendMessageToClient("DeviceToBeAllocated:CFW:" + QString::number(ConnectedFILTERList[i]) + ":" + QString::fromUtf8(device->getDeviceName()));
                }
            }
        }
    }

    Logger::Log("Each Device Only Has One:" + std::to_string(EachDeviceOne), LogLevel::INFO, DeviceType::MAIN);
    if (EachDeviceOne)
    {
        // AfterDeviceConnect();
    }
    else
    {
        emit wsThread->sendMessageToClient("ShowDeviceAllocationWindow");
    }
    emit wsThread->sendMessageToClient("AddDeviceType:" + systemdevicelist.system_devices[driverCode].Description);
    emit wsThread->sendMessageToClient("ConnectDriverSuccess:" + DriverName);
}
void MainWindow::DisconnectDevice(MyClient *client, QString DeviceName, QString DeviceType)
{
    if (DeviceName == "" || DeviceType == "")
    {
        Logger::Log("DisconnectDevice | DeviceName(" + DeviceName.toStdString() + ") or DeviceType(" + DeviceType.toStdString() + ") is Null", LogLevel::WARNING, DeviceType::MAIN);
        return;
    }
    if (DeviceType == "Not Bind Device")
    {
        emit wsThread->sendMessageToClient("DisconnectDriverSuccess:" + DeviceType);
        return;
    }
    if (DeviceType == "Guider")

        {
        // 停止导星
        call_phd_StopLooping();
        isGuiding = false;
        emit wsThread->sendMessageToClient("GuiderSwitchStatus:false");
        isGuiderLoopExp = false;
        emit wsThread->sendMessageToClient("GuiderLoopExpStatus:false");
        emit wsThread->sendMessageToClient("GuiderUpdateStatus:0");
        sleep(3);
    }

    Logger::Log("DisconnectDevice | Disconnect " + DeviceType.toStdString() + " Device(" + DeviceName.toStdString() + ") start...", LogLevel::INFO, DeviceType::MAIN);
    int num = 0;
    int thisDriverhasDevice = 0;
    bool driverIsUsed = false;
    bool disconnectsuccess = true;
    QString disconnectdriverName;
    QVector<QString> NeedDisconnectDeviceNameList;

    for (int i = 0; i < client->GetDeviceCount(); i++)
    {
        if (client->GetDeviceFromList(i)->getDeviceName() == DeviceName)
        {
            client->disconnectDevice(client->GetDeviceFromList(i)->getDeviceName());
            while (client->GetDeviceFromList(i)->isConnected())
            {
                Logger::Log("DisconnectDevice | Waiting for disconnect finish...", LogLevel::INFO, DeviceType::MAIN);
                sleep(1);
                num++;
                if (num > 5)
                {
                    Logger::Log("DisconnectDevice | Disconnect " + DeviceType.toStdString() + " Device(" + DeviceName.toStdString() + ") failed.", LogLevel::WARNING, DeviceType::MAIN);
                    disconnectsuccess = false;
                    break;
                }
            }
            if (!disconnectsuccess)
            {
                break;
            }
            Logger::Log("DisconnectDevice | Disconnect " + DeviceType.toStdString() + " Device(" + DeviceName.toStdString() + ") success.", LogLevel::INFO, DeviceType::MAIN);

            emit wsThread->sendMessageToClient("DisconnectDriverSuccess:" + DeviceType);
            break;
        }
    }
    if (!disconnectsuccess)
    {
        Logger::Log("DisconnectDevice | Disconnect " + DeviceType.toStdString() + " Device(" + DeviceName.toStdString() + ") failed.", LogLevel::WARNING, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("DisconnectDriverFail:" + DeviceType);
    }

    if (DeviceType == "MainCamera")
    {
        dpMainCamera = NULL;
        disconnectdriverName = systemdevicelist.system_devices[20].DriverIndiName;
        systemdevicelist.system_devices[20].isConnect = false;
        systemdevicelist.system_devices[20].isBind = false;
        systemdevicelist.system_devices[20].DeviceIndiName = "";
        systemdevicelist.system_devices[20].DeviceIndiGroup = -1;
        systemdevicelist.system_devices[20].DriverFrom = "";
        systemdevicelist.system_devices[20].dp = NULL;
    }
    else if (DeviceType == "Guider")
    {
        dpGuider = NULL;
        disconnectdriverName = systemdevicelist.system_devices[1].DriverIndiName;
        systemdevicelist.system_devices[1].isConnect = false;
        systemdevicelist.system_devices[1].isBind = false;
        systemdevicelist.system_devices[1].DeviceIndiName = "";
        systemdevicelist.system_devices[1].DeviceIndiGroup = -1;
        systemdevicelist.system_devices[1].DriverFrom = "";
        systemdevicelist.system_devices[1].dp = NULL;
    }
    else if (DeviceType == "PoleCamera")
    {
        dpPoleScope = NULL;
        disconnectdriverName = systemdevicelist.system_devices[2].DriverIndiName;
        systemdevicelist.system_devices[2].isConnect = false;
        systemdevicelist.system_devices[2].isBind = false;
        systemdevicelist.system_devices[2].DeviceIndiName = "";
        systemdevicelist.system_devices[2].DeviceIndiGroup = -1;
        systemdevicelist.system_devices[2].DriverFrom = "";
        systemdevicelist.system_devices[2].dp = NULL;
    }
    else if (DeviceType == "Mount")
    {
        dpMount = NULL;
        disconnectdriverName = systemdevicelist.system_devices[0].DriverIndiName;
        systemdevicelist.system_devices[0].isConnect = false;
        systemdevicelist.system_devices[0].isBind = false;
        systemdevicelist.system_devices[0].DeviceIndiName = "";
        systemdevicelist.system_devices[0].DeviceIndiGroup = -1;
        systemdevicelist.system_devices[0].DriverFrom = "";
        systemdevicelist.system_devices[0].dp = NULL;
    }
    else if (DeviceType == "Focuser")
    {
        dpFocuser = NULL;
        disconnectdriverName = systemdevicelist.system_devices[22].DriverIndiName;
        systemdevicelist.system_devices[22].isConnect = false;
        systemdevicelist.system_devices[22].isBind = false;
        systemdevicelist.system_devices[22].DeviceIndiName = "";
        systemdevicelist.system_devices[22].DeviceIndiGroup = -1;
        systemdevicelist.system_devices[22].DriverFrom = "";
        systemdevicelist.system_devices[22].dp = NULL;
    }
    else if (DeviceType == "CFW")
    {
        dpCFW = NULL;
        disconnectdriverName = systemdevicelist.system_devices[21].DriverIndiName;
        systemdevicelist.system_devices[21].isConnect = false;
        systemdevicelist.system_devices[21].isBind = false;
        systemdevicelist.system_devices[21].DeviceIndiName = "";
        systemdevicelist.system_devices[21].DeviceIndiGroup = -1;
        systemdevicelist.system_devices[21].DriverFrom = "";
        systemdevicelist.system_devices[21].dp = NULL;
    }

    QStringList SelectedCameras;
    // QStringList SelectedCameras = Tools::getCameraNumFromSystemDeviceList(systemdevicelist);
    for (int i = 0; i < systemdevicelist.system_devices.size(); i++)
    {
        // qInfo() << "systemdevicelist.system_devices["<< i <<"].DriverIndiName:" << systemdevicelist.system_devices[i].DriverIndiName;
        // qInfo() << "systemdevicelist.system_devices["<< i <<"].Description:" << systemdevicelist.system_devices[i].Description;
        if (systemdevicelist.system_devices[i].Description != "" && systemdevicelist.system_devices[i].isConnect == true)
        {
            SelectedCameras.push_back(systemdevicelist.system_devices[i].Description);
        }
    }

    for (auto it = ConnectedDevices.begin(); it != ConnectedDevices.end();)
    {
        if (it->DeviceType == DeviceType)
        {
            it = ConnectedDevices.erase(it); // 删除元素并更新迭代器
        }
        else
        {
            ++it; // 仅在不删除时递增迭代器
        }
    }

    if (SelectedCameras.size() == 0)
    {
        Logger::Log("DisconnectDevice | No Device Connected, need to clear all devices", LogLevel::INFO, DeviceType::MAIN);
        disconnectIndiServer(indi_Client);
        // ClearSystemDeviceList();
        clearConnectedDevices();

        initINDIServer();
        initINDIClient();
        // Tools::InitSystemDeviceList();
        // Tools::initSystemDeviceList(systemdevicelist);
        getLastSelectDevice();
        Tools::printSystemDeviceList(systemdevicelist);
        if (wsThread != nullptr) emit wsThread->sendMessageToClient("DisconnectDriverSuccess:all");
        return;
    }

    // Tools::startIndiDriver(disconnectdriverName);
    sleep(1);

    for (int i = 0; i < systemdevicelist.system_devices.size(); i++)
    {
        if (systemdevicelist.system_devices[i].DriverIndiName == disconnectdriverName)
        {
            if (systemdevicelist.system_devices[i].isConnect == true)
            {
                driverIsUsed = true;
                thisDriverhasDevice++;
            }
        }
    }
    num = 0;
    if (thisDriverhasDevice >= 1 && client->GetDeviceFromListWithName(DeviceName.toStdString())->isConnected() == false)
    {
        client->connectDevice(DeviceName.toStdString().c_str());
        Logger::Log("DisconnectDevice | This Driver has more than one device is using, need to reconnect device(" + DeviceName.toStdString() + ")", LogLevel::INFO, DeviceType::MAIN);
        while (!client->GetDeviceFromListWithName(DeviceName.toStdString())->isConnected())
        {
            Logger::Log("DisconnectDevice | Waiting for connect finish...", LogLevel::INFO, DeviceType::MAIN);
            sleep(1);
            num++;
            if (num > 10)
            {
                Logger::Log("DisconnectDevice | Reconnect Device(" + DeviceName.toStdString() + ") failed.", LogLevel::WARNING, DeviceType::MAIN);
                break;
            }
        }
        if (client->GetDeviceFromListWithName(DeviceName.toStdString())->isConnected())
        {
            Logger::Log("DisconnectDevice | Reconnect Device(" + DeviceName.toStdString() + ") success.", LogLevel::INFO, DeviceType::MAIN);
        }
        else
        {
            Logger::Log("DisconnectDevice | Reconnect Device(" + DeviceName.toStdString() + ") failed.", LogLevel::WARNING, DeviceType::MAIN);
        }
        emit wsThread->sendMessageToClient("disconnectDevicehasortherdevice:" + disconnectdriverName);
    }
    else
    {
        Tools::stopIndiDriver(disconnectdriverName);
        int index = ConnectDriverList.indexOf(disconnectdriverName);
        if (index != -1)
        {                                      // 如果找到了
            ConnectDriverList.removeAt(index); // 从列表中删除
            Logger::Log("Driver removed successfully: " + disconnectdriverName.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        }
        else
        {
            Logger::Log("Driver not found in list: " + disconnectdriverName.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
        }
    }
    emit wsThread->sendMessageToClient("deleteDeviceAllocationList:" + DeviceName);
}

void MainWindow::loadSelectedDriverList()
{
    // 1. 检查 systemdevicelist.system_devices 是否为空
    if (systemdevicelist.system_devices.empty())
    {
        Logger::Log("loadSelectedDriverList | system_devices is empty", LogLevel::ERROR, DeviceType::MAIN);
        return;
    }

    // 2. 检查 wsThread 是否为空
    if (wsThread == nullptr)
    {
        Logger::Log("loadSelectedDriverList | wsThread is null", LogLevel::ERROR, DeviceType::MAIN);
        return;
    }

    QString order = "SelectedDriverList";

    // 3. 使用安全的范围检查
    try
    {
        for (int i = 0; i < systemdevicelist.system_devices.size(); i++)
        {
            // 4. 检查索引是否有效
            if (i >= systemdevicelist.system_devices.size())
            {
                Logger::Log("loadSelectedDriverList | Index out of bounds: " + std::to_string(i), LogLevel::ERROR, DeviceType::MAIN);
                break;
            }

            // 5. 检查 Description 是否有效
            if (!systemdevicelist.system_devices[i].Description.isEmpty())
            {
                QString description = systemdevicelist.system_devices[i].Description;
                QString driverName = systemdevicelist.system_devices[i].DriverIndiName;

                // 6. 检查字符串是否有效
                if (!description.isEmpty() && !driverName.isEmpty())
                {
                    order += ":" + description + ":" + driverName;
                    Logger::Log("loadSelectedDriverList | Added device: " + description.toStdString() + " - " + driverName.toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
                }
            }
        }

        // 7. 确保 wsThread 和 sendMessageToClient 方法存在
        if (wsThread != nullptr)
        {
            Logger::Log("loadSelectedDriverList | Sending message: " + order.toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
            emit wsThread->sendMessageToClient(order);
        }
    }
    catch (const std::exception &e)
    {
        Logger::Log("loadSelectedDriverList | Exception caught: " + std::string(e.what()), LogLevel::ERROR, DeviceType::MAIN);
    }
    catch (...)
    {
        Logger::Log("loadSelectedDriverList | Unknown exception caught", LogLevel::ERROR, DeviceType::MAIN);
    }
}

void MainWindow::loadBindDeviceTypeList()
{
    QString order = "BindDeviceTypeList";
    for (int i = 0; i < systemdevicelist.system_devices.size(); i++)
    {
        if (systemdevicelist.system_devices[i].Description != "" && systemdevicelist.system_devices[i].isConnect == true)
        {
            order += ":" + systemdevicelist.system_devices[i].Description + ":" + systemdevicelist.system_devices[i].DeviceIndiName + ":" + systemdevicelist.system_devices[i].DriverIndiName + ":" + (systemdevicelist.system_devices[i].isBind ? "true" : "false");
            if (systemdevicelist.system_devices[i].Description == "MainCamera" && systemdevicelist.system_devices[i].isBind)
            {
                emit wsThread->sendMessageToClient("MainCameraSize:" + QString::number(glMainCCDSizeX) + ":" + QString::number(glMainCCDSizeY));
                emit wsThread->sendMessageToClient("MainCameraOffsetRange:" + QString::number(glOffsetMin) + ":" + QString::number(glOffsetMax) + ":" + QString::number(glOffsetValue));
                emit wsThread->sendMessageToClient("MainCameraGainRange:" + QString::number(glGainMin) + ":" + QString::number(glGainMax) + ":" + QString::number(glGainValue));

                QString CFWname;
                indi_Client->getCFWSlotName(dpMainCamera, CFWname);
                if (CFWname != "")
                {
                    Logger::Log("LoadBindDeviceTypeList | get CFW Slot Name: " + CFWname.toStdString(), LogLevel::INFO, DeviceType::MAIN);
                    // emit wsThread->sendMessageToClient("ConnectSuccess:CFW:" + CFWname +" (on camera)");
                    isFilterOnCamera = true;
                    order += ":CFW:" + CFWname + " (on camera)" + ":" + systemdevicelist.system_devices[i].DriverIndiName + ":" + (systemdevicelist.system_devices[i].isBind ? "true" : "false");
                    int min, max, pos;
                    indi_Client->getCFWPosition(dpMainCamera, pos, min, max);
                    Logger::Log("LoadBindDeviceTypeList | getCFWPosition: " + std::to_string(min) + ", " + std::to_string(max) + ", " + std::to_string(pos), LogLevel::INFO, DeviceType::MAIN);
                    emit wsThread->sendMessageToClient("CFWPositionMax:" + QString::number(max));
                }
            }
        }
    }
    Logger::Log("LoadBindDeviceTypeList | Bind Device Type List:" + order.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    emit wsThread->sendMessageToClient(order);
}

void MainWindow::loadBindDeviceList(MyClient *client)
{
    QString order = "BindDeviceList";

    // 修复：防御性编程，防止空指针和非法访问导致段错误
    if (client == nullptr)
    {
        Logger::Log("LoadBindDeviceList | client is nullptr", LogLevel::ERROR, DeviceType::MAIN);
        emit wsThread->sendMessageToClient(order);
        return;
    }

    int deviceCount = client->GetDeviceCount();
    if (deviceCount <= 0)
    {
        Logger::Log("LoadBindDeviceList | no devices in client list", LogLevel::INFO, DeviceType::MAIN);
        emit wsThread->sendMessageToClient(order);
        return;
    }

    for (int i = 0; i < deviceCount; i++)
    {
        INDI::BaseDevice *device = client->GetDeviceFromList(i);
        if (device == nullptr)
        {
            Logger::Log("LoadBindDeviceList | Device at index " + std::to_string(i) + " is nullptr", LogLevel::WARNING, DeviceType::MAIN);
            continue;
        }

        const char *name = device->getDeviceName();
        if (!name)
        {
            Logger::Log("LoadBindDeviceList | Device at index " + std::to_string(i) + " has null name pointer", LogLevel::WARNING, DeviceType::MAIN);
            continue;
        }

        QString qName = QString::fromUtf8(name);
        if (device->isConnected() && !qName.isEmpty())
        {
            order += ":" + qName + ":" + QString::number(i);
        }
    }

    Logger::Log("LoadBindDeviceList | Bind Device List:" + order.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    emit wsThread->sendMessageToClient(order);
}

void MainWindow::loadSDKVersionAndUSBSerialPath()
{
    QString order = "SDKVersionAndUSBSerialPath";
    if (dpMainCamera != NULL)
    {
        QString sdkVersion = "null";
        indi_Client->getCCDSDKVersion(dpMainCamera, sdkVersion);
        order += ":MainCamera:" + sdkVersion + ":null";
    }
    if (dpGuider != NULL)
    {
        QString sdkVersion = "null";
        indi_Client->getCCDSDKVersion(dpGuider, sdkVersion);
        order += ":Guider:" + sdkVersion + ":null";
    }
    if (dpFocuser != NULL)
    {
        QString sdkVersion = "null";
        indi_Client->getFocuserSDKVersion(dpFocuser, sdkVersion);
        QString DevicePort = "null";
        indi_Client->getDevicePort(dpFocuser, DevicePort);
        order += ":Focuser:" + sdkVersion + ":" + DevicePort;
    }
    // if (dpCFW != NULL)
    // {
    //     QString sdkVersion;
    //     indi_Client->getSDKVersion(dpCFW, sdkVersion);
    //     QString usbSerialPath;
    //     indi_Client->getUSBSerialPath(dpCFW, usbSerialPath);
    //     order += ":CFW:" + sdkVersion + ":" + usbSerialPath;
    // }
    if (dpMount != NULL)
    {
        QString sdkVersion;
        indi_Client->getMountInfo(dpMount, sdkVersion);
        QString usbSerialPath;
        indi_Client->getDevicePort(dpMount, usbSerialPath);
        order += ":Mount:" + sdkVersion + ":" + usbSerialPath;
    }
    emit wsThread->sendMessageToClient(order);
    Logger::Log("LoadSDKVersionAndUSBSerialPath | SDKVersionAndUSBSerialPath:" + order.toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
}

// 串口通信列表
QStringList MainWindow::getConnectedSerialPorts()
{
    QStringList activeSerialPortNames;
    const auto infos = QSerialPortInfo::availablePorts();
    for (const QSerialPortInfo &info : infos)
    {
        // 使用系统路径，便于直接设置到 INDI 设备端口，例如 /dev/ttyUSB0
        // 不再强制尝试打开端口，以免过滤掉权限/占用导致暂时无法打开但仍可被用户选择的端口
        activeSerialPortNames.append(info.systemLocation());
    }
    return activeSerialPortNames;
}

QString MainWindow::resolveSerialPort(const QString &symbolicLink)
{
    QFileInfo fileInfo(symbolicLink);
    if (fileInfo.isSymLink())
    {
        QString target = fileInfo.symLinkTarget();
        Logger::Log("ResolveSerialPort | real port path:" + target.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        return target;
    }
    else
    {
        Logger::Log("ResolveSerialPort | provided path is not a symbolic link", LogLevel::WARNING, DeviceType::MAIN);
        return QString();
    }
}

void MainWindow::sendSerialPortOptions(const QString &driverType)
{
    if (!wsThread)
        return;

    // 仅支持 Mount / Focuser 两类串口设备
    if (driverType != "Mount" && driverType != "Focuser")
        return;

    // 当前可用串口路径列表（全部是真实存在的串口节点）
    QStringList ports = getConnectedSerialPorts();

    // 当前设备正在使用的串口（若已连接），或前端最近一次选择的覆盖串口
    QString currentPort;
    if (driverType == "Mount")
    {
        if (dpMount != nullptr)
        {
            indi_Client->getDevicePort(dpMount, currentPort);
        }
        if (currentPort.isEmpty())
        {
            currentPort = mountSerialPortOverride;
        }
    }
    else if (driverType == "Focuser")
    {
        if (dpFocuser != nullptr)
        {
            indi_Client->getDevicePort(dpFocuser, currentPort);
        }
        if (currentPort.isEmpty())
        {
            currentPort = focuserSerialPortOverride;
        }
    }

    // 组装带“真实路径 -> 友好名称(by-id)”的项：
    // 每一项格式为：<portPath>-><displayName>，前端再解析
    QString payload = "SerialPortOptions:" + driverType + ":" + currentPort;
    for (const QString &p : ports)
    {
        QString displayName = p;
        QFileInfo fi(p);
        QString ttyName = fi.fileName(); // 例如 ttyUSB0

        // 若能找到 /dev/serial/by-id 的符号链接，则使用其文件名作为显示名
        QStringList byIdLinks = getByIdLinksForTty(ttyName);
        if (!byIdLinks.isEmpty())
        {
            QFileInfo linkInfo(byIdLinks.first());
            displayName = linkInfo.fileName(); // 只显示 by-id 的名字，更易识别设备
        }

        payload += ":" + p + "->" + displayName;
    }

    Logger::Log("sendSerialPortOptions | " + payload.toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
    emit wsThread->sendMessageToClient(payload);
}

QStringList MainWindow::findLinkToTtyDevice(const QString &directoryPath, const QString &ttyDevice)
{
    QString targetDevice = "/dev/" + ttyDevice; // 构建完整的设备路径
    QStringList foundLinks;

    // 使用 QDirIterator 递归遍历目录和子目录
    QDirIterator it(directoryPath, QDir::Files | QDir::System | QDir::NoDotAndDotDot, QDirIterator::Subdirectories);
    while (it.hasNext())
    {
        it.next();
        QFileInfo fileInfo = it.fileInfo();
        if (fileInfo.isSymLink())
        {
            QString target = fileInfo.symLinkTarget();
            // 如果符号链接是相对路径，需要将其转换为绝对路径
            if (QDir::isRelativePath(target))
            {
                target = fileInfo.absoluteDir().absoluteFilePath(target);
            }
            // 检查符号链接的目标是否是指定的 tty 设备
            if (target == targetDevice)
            {
                foundLinks.append(fileInfo.absoluteFilePath());
                Logger::Log("FindLinkToTtyDevice | found link:" + fileInfo.absoluteFilePath().toStdString() + " -> " + target.toStdString(), LogLevel::INFO, DeviceType::MAIN);
            }
        }
    }
    // 返回找到的所有符号链接
    return foundLinks;
}

bool MainWindow::areFilesInSameDirectory(const QString &path1, const QString &path2)
{
    QFileInfo fileInfo1(path1);
    QFileInfo fileInfo2(path2);

    // 获取两个文件的目录路径
    QString dir1 = fileInfo1.absolutePath();
    QString dir2 = fileInfo2.absolutePath();

    // 比较目录路径是否相同
    return (dir1 == dir2);
}

QStringList MainWindow::getByIdLinksForTty(const QString &ttyDevice)
{
    QStringList results;
    QString baseDir = "/dev/serial/by-id";
    QDir dir(baseDir);
    if (!dir.exists())
    {
        return results;
    }

    QFileInfoList entryList = dir.entryInfoList(QDir::Files | QDir::System | QDir::NoDotAndDotDot);
    QString targetDevice = "/dev/" + ttyDevice;
    for (const QFileInfo &entry : entryList)
    {
        if (!entry.isSymLink())
        {
            continue;
        }
        QString target = entry.symLinkTarget();
        if (QDir::isRelativePath(target))
        {
            // 归一化相对路径为绝对路径
            target = entry.absoluteDir().absoluteFilePath(target);
        }
        QString normalizedTarget = QDir::cleanPath(target);
        if (normalizedTarget == targetDevice)
        {
            results.append(entry.absoluteFilePath());
            Logger::Log("getByIdLinksForTty | found by-id link:" + entry.absoluteFilePath().toStdString() + " -> " + normalizedTarget.toStdString(), LogLevel::INFO, DeviceType::MAIN);
        }
    }
    return results;
}

static int scoreByIdLinkForType(const QString &fileNameLower, const QString &driverType)
{
    // 简单关键字打分，匹配越多分越高
    int score = 0;
    if (driverType == "Focuser")
    {
        if (fileNameLower.contains("gigadevice")) score += 2;
        if (fileNameLower.contains("gd32")) score += 2;
        if (fileNameLower.contains("cdc_acm")) score += 1;
        if (fileNameLower.contains("acm")) score += 1;
    }
    else if (driverType == "Mount")
    {
        if (fileNameLower.contains("1a86")) score += 2;          // WCH/CH34x VID
        if (fileNameLower.contains("usb_serial")) score += 2;    // CH34x 常见 by-id
        if (fileNameLower.contains("ch34")) score += 2;
        if (fileNameLower.contains("wch")) score += 1;
        if (fileNameLower.contains("ttyusb")) score += 1;        // 兜底弱信号
    }
    return score;
}

bool MainWindow::isByIdLinkForDriverType(const QString &symlinkPath, const QString &driverType)
{
    QFileInfo fi(symlinkPath);
    QString nameLower = fi.fileName().toLower();
    return scoreByIdLinkForType(nameLower, driverType) > 0;
}

QString MainWindow::selectBestByIdLink(const QStringList &links, const QString &driverType)
{
    int bestScore = -1;
    QString best;
    for (const QString &link : links)
    {
        QFileInfo fi(link);
        QString nameLower = fi.fileName().toLower();
        int s = scoreByIdLinkForType(nameLower, driverType);
        if (s > bestScore)
        {
            bestScore = s;
            best = link;
        }
    }
    return best;
}

void MainWindow::onParseInfoEmitted(const QString &message)
{
    emit wsThread->sendMessageToClient("ParseInfoEmitted:" + message);
}

void MainWindow::disconnectDevice(const QString &deviceName, const QString &description)
{
    for (int i = 0; i < indi_Client->GetDeviceCount(); i++)
    {
        if (indi_Client->GetDeviceFromList(i)->getDeviceName() == deviceName)
        {
            indi_Client->disconnectDevice(deviceName.toStdString().c_str());
            int num = 0;
            bool disconnectSuccess = true;
  
            Logger::Log(deviceName.toStdString() + " disconnected successfully.", LogLevel::INFO, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("DisconnectDriverSuccess:" + description);
            emit wsThread->sendMessageToClient("deleteDeviceAllocationList:" + deviceName);

            // 若为赤道仪或电调，在断开后提示前端弹出串口选择 UI，方便下次连接前重新匹配
            if (description == "Mount" || description == "Focuser")
            {
                sendSerialPortOptions(description);
                emit wsThread->sendMessageToClient("RequestSerialPortSelection:" + description);
            }
            break;
        }
    }
}

void MainWindow::disconnectDriver(QString Driver)
{
    Logger::Log("Starting to disconnect driver: " + Driver.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    for (const auto &device : systemdevicelist.system_devices)
    {
        if (device.Description != "" && device.DriverIndiName == Driver && device.isConnect)
        {
            disconnectDevice(device.DeviceIndiName, device.Description);
            if (device.Description == "MainCamera")
            {
                if (glMainCameraStatu == "Exposuring")
                {
                    INDI_AbortCapture();
                }
                systemdevicelist.system_devices[20].isConnect = false;
                systemdevicelist.system_devices[20].DeviceIndiName = "";
                systemdevicelist.system_devices[20].DeviceIndiGroup = -1;
                systemdevicelist.system_devices[20].DriverFrom = "";
                systemdevicelist.system_devices[20].isBind = false;
                systemdevicelist.system_devices[20].dp = NULL;
            }
            if (device.Description == "Guider")
            {
                if (isGuiderLoopExp)
                {
                    call_phd_StopLooping();
                    InitPHD2();
                }
                systemdevicelist.system_devices[1].isConnect = false;
                systemdevicelist.system_devices[1].DeviceIndiName = "";
                systemdevicelist.system_devices[1].DeviceIndiGroup = -1;
                systemdevicelist.system_devices[1].DriverFrom = "";
                systemdevicelist.system_devices[1].isBind = false;
                systemdevicelist.system_devices[1].dp = NULL;
            }
            for (auto it = ConnectedDevices.begin(); it != ConnectedDevices.end();)
            {
                if (it->DeviceType == device.Description)
                {
                    it = ConnectedDevices.erase(it); // 删除元素并更新迭代器
                }
                else
                {
                    ++it; // 仅在不删除时递增迭代器
                }
            }
        }
    }
    Tools::stopIndiDriver(Driver);
    int index = ConnectDriverList.indexOf(Driver);
    if (index != -1)
    {                                      // 如果找到了
        ConnectDriverList.removeAt(index); // 从列表中删除
        Logger::Log("Driver removed successfully: " + Driver.toStdString(), LogLevel::INFO, DeviceType::MAIN);
    }
    else
    {
        Logger::Log("Driver not found in list: " + Driver.toStdString(), LogLevel::WARNING, DeviceType::MAIN);
    }

    Logger::Log("Driver disconnected: " + Driver.toStdString(), LogLevel::INFO, DeviceType::MAIN);
}

void MainWindow::focusLoopShooting(bool isLoop)
{
    if (isLoop)
    {
        isFocusLoopShooting = true;
        if (glMainCameraStatu == "Exposuring")
        {
            emit wsThread->sendMessageToClient("startFocusLoopFailed:Wait Take Picture Finish!");
            isFocusLoopShooting = false;
            return;
        }
        if (dpMainCamera == NULL)
        {
            emit wsThread->sendMessageToClient("startFocusLoopFailed:MainCamera is not connected!");
            isFocusLoopShooting = false;
            return;
        }
        FocusingLooping();
    }
    else
    {
        isFocusLoopShooting = false;
        hasPendingRoiUpdate = false;
        if (glMainCameraStatu == "Exposuring" && dpMainCamera != NULL)
        {
            INDI_AbortCapture();
        }
    }
}
void MainWindow::bin_image(double *input, long width, long height, double *output, long *out_w, long *out_h)
{
    *out_w = width / BIN_SIZE;
    *out_h = height / BIN_SIZE;

    for (long y = 0; y < *out_h; y++)
    {
        for (long x = 0; x < *out_w; x++)
        {
            double sum = 0.0;
            for (int dy = 0; dy < BIN_SIZE; dy++)
            {
                for (int dx = 0; dx < BIN_SIZE; dx++)
                {
                    long ix = x * BIN_SIZE + dx;
                    long iy = y * BIN_SIZE + dy;
                    sum += input[iy * width + ix];
                }
            }
            output[y * (*out_w) + x] = sum / (BIN_SIZE * BIN_SIZE);
        }
    }
}

void MainWindow::process_hdu(fitsfile *infptr, fitsfile *outfptr, int hdunum, int *status)
{
    fits_movabs_hdu(infptr, hdunum, NULL, status);

    int bitpix, naxis;
    long naxes[2] = {1, 1};
    fits_get_img_param(infptr, 2, &bitpix, &naxis, naxes, status);

    if (naxis != 2)
    {
        printf("HDU %d skipped (not 2D image).\n", hdunum);
        return;
    }

    long width = naxes[0], height = naxes[1];
    long npixels = width * height;
    double *img = (double *)malloc(npixels * sizeof(double));
    if (!img)
    {
        printf("Memory error.\n");
        exit(1);
    }

    long fpixel[2] = {1, 1};
    fits_read_pix(infptr, TDOUBLE, fpixel, npixels, NULL, img, NULL, status);

    long out_w, out_h;
    long dims[2] = {out_w, out_h};
    long out_pixels = (width / BIN_SIZE) * (height / BIN_SIZE);
    double *binned = (double *)malloc(out_pixels * sizeof(double));
    bin_image(img, width, height, binned, &out_w, &out_h);

    // 创建输出图像
    fits_create_img(outfptr, DOUBLE_IMG, 2, dims, status);
    fits_write_img(outfptr, TDOUBLE, 1, out_w * out_h, binned, status);

    free(img);
    free(binned);
}
int MainWindow::process_fixed()
{
    const char *infile = "/dev/shm/ccd_simulator_original.fits"; // 输入文件路径
    const char *outfile = "!/dev/shm/ccd_simulator_binned.fits"; // 输出文件路径
    // const char *outfile = "!merged_output.fits";  // 带 '!' 前缀，自动覆盖

    fitsfile *infptr = NULL, *outfptr = NULL;
    int status = 0, hdunum = 0, hdutype = 0;

    fits_open_file(&infptr, infile, READONLY, &status);
    if (status)
    {
        fits_report_error(stderr, status);
        return status;
    }

    fits_create_file(&outfptr, outfile, &status);
    if (status)
    {
        fits_report_error(stderr, status);
        fits_close_file(infptr, &status);
        return status;
    }

    fits_get_num_hdus(infptr, &hdunum, &status);
    for (int i = 1; i <= hdunum && status == 0; i++)
    {
        fits_movabs_hdu(infptr, i, &hdutype, &status);
        if (hdutype == IMAGE_HDU)
        {
            process_hdu(infptr, outfptr, i, &status);
        }
        else
        {
            fits_copy_hdu(infptr, outfptr, 0, &status);
        }
    }

    fits_close_file(infptr, &status);
    fits_close_file(outfptr, &status);

    if (status)
    {
        fits_report_error(stderr, status);
        return status;
    }

    printf("合并覆盖完成：ccd_simulator_binned.fits\n");
    return 0;
}


void MainWindow::saveFitsAsJPG(QString filename, bool ProcessBin)
{
    cv::Mat image;
    // 读取FITS文件
    Tools::readFits(filename.toLocal8Bit().constData(), image);

    QList<FITSImage::Star> stars = Tools::FindStarsByFocusedCpp(true, true);
    currentSelectStarPosition = selectStar(stars);

    emit wsThread->sendMessageToClient("FocusMoveDone:" + QString::number(FocuserControl_getPosition()) + ":" + QString::number(roiAndFocuserInfo["SelectStarHFR"]));
    emit wsThread->sendMessageToClient("setSelectStarPosition:" + QString::number(roiAndFocuserInfo["SelectStarX"]) + ":" + QString::number(roiAndFocuserInfo["SelectStarY"]) + ":" + QString::number(roiAndFocuserInfo["SelectStarHFR"]));
    emit wsThread->sendMessageToClient("addFwhmNow:" + QString::number(roiAndFocuserInfo["SelectStarHFR"]));
    Logger::Log("saveFitsAsJPG | 星点位置更新为 x:" + std::to_string(roiAndFocuserInfo["SelectStarX"]) + ",y:" + std::to_string(roiAndFocuserInfo["SelectStarY"]) + ",HFR:" + std::to_string(roiAndFocuserInfo["SelectStarHFR"]), LogLevel::INFO, DeviceType::FOCUSER);

    // 判断当前相机是否为彩色相机
    bool isColor = !(MainCameraCFA == "" || MainCameraCFA == "null");
    cv::Mat originalImage16;
    if (image.type() == CV_8UC1 || image.type() == CV_8UC3 || image.type() == CV_16UC1)
    {
        originalImage16 = Tools::convert8UTo16U_BayerSafe(image, false);
        Logger::Log("saveFitsAsJPG | image size:" + std::to_string(image.cols) + "x" + std::to_string(image.rows), LogLevel::INFO, DeviceType::FOCUSER);
        image.release();
    }
    else
    {
        Logger::Log("The current image data type is not supported for processing.", LogLevel::WARNING, DeviceType::CAMERA);
        image.release();
        originalImage16.release();
        return;
    }
    Logger::Log("saveFitsAsJPG | image16 size:" + std::to_string(originalImage16.cols) + "x" + std::to_string(originalImage16.rows), LogLevel::INFO, DeviceType::FOCUSER);

    // 中值滤波
    Logger::Log("Starting median blur...", LogLevel::INFO, DeviceType::CAMERA);
    cv::medianBlur(originalImage16, originalImage16, 3);
    Logger::Log("Median blur applied successfully.", LogLevel::INFO, DeviceType::CAMERA);

    cv::Mat image16;
    if (ProcessBin && glMainCameraBinning != 1)
    {
        // 使用新的Mat版本的PixelsDataSoftBin_Bayer函数
        if (MainCameraCFA == "RGGB" || MainCameraCFA == "RG")
        {
            image16 = Tools::PixelsDataSoftBin_Bayer(originalImage16, glMainCameraBinning, glMainCameraBinning, BAYER_RGGB);
        }
        else if (MainCameraCFA == "BGGR" || MainCameraCFA == "BG")
        {
            image16 = Tools::PixelsDataSoftBin_Bayer(originalImage16, glMainCameraBinning, glMainCameraBinning, BAYER_BGGR);
        }
        else if (MainCameraCFA == "GRBG" || MainCameraCFA == "GR")
        {
            image16 = Tools::PixelsDataSoftBin_Bayer(originalImage16, glMainCameraBinning, glMainCameraBinning, BAYER_GRBG);
        }
        else if (MainCameraCFA == "GBRG" || MainCameraCFA == "GB")
        {
            image16 = Tools::PixelsDataSoftBin_Bayer(originalImage16, glMainCameraBinning, glMainCameraBinning, BAYER_GBRG);
        }
        else
        {
            image16 = Tools::processMatWithBinAvg(originalImage16, glMainCameraBinning, glMainCameraBinning, isColor, true);
        }
    }
    else
    {
        image16 = originalImage16.clone();
    }
    Logger::Log("saveFitsAsJPG | ROI_x:" + std::to_string(roiAndFocuserInfo["ROI_x"]) + ", ROI_y:" + std::to_string(roiAndFocuserInfo["ROI_y"]), LogLevel::INFO, DeviceType::FOCUSER);
    Logger::Log("saveFitsAsJPG | image16 size:" + std::to_string(image16.cols) + "x" + std::to_string(image16.rows), LogLevel::INFO, DeviceType::FOCUSER);
    originalImage16.release();

    // 删除所有以 "focuserPicture_" 开头的文件
    for (const auto &entry : std::filesystem::directory_iterator(vueDirectoryPath))
    {
        if (entry.path().filename().string().rfind("focuserPicture_", 0) == 0)
        {
            std::filesystem::remove(entry.path());
        }
    }

    // 保存新的图像带有唯一ID的文件名
    time_t now = time(0);
    tm *ltm = localtime(&now);
    std::string fileName = "focuserPicture_" + std::to_string(ltm->tm_hour) + std::to_string(ltm->tm_min) + std::to_string(ltm->tm_sec) + ".bin";
    std::string filePath = vueDirectoryPath + fileName;
    std::ofstream outFile(filePath, std::ios::binary);

    // 检查文件是否成功打开
    if (!outFile)
    {
        Logger::Log("Failed to open file: " + filePath, LogLevel::WARNING, DeviceType::FOCUSER);
        if (isFocusLoopShooting)
        {
            FocusingLooping();
        }
        return;
    }

    // 将图像数据写入到文件中
    outFile.write(reinterpret_cast<const char *>(image16.data), image16.total() * image16.elemSize());

    // 检查是否成功写入
    if (!outFile)
    {
        Logger::Log("Failed to write to file: " + filePath, LogLevel::ERROR, DeviceType::FOCUSER);
        if (isFocusLoopShooting)
        {
            FocusingLooping();
        }
        return;
    }

    outFile.close();

    bool saved = true;

    try
    {
        fs::path dirPath = fs::path(vueImagePath);
        // 遍历目录中的所有条目
        for (const auto &entry : fs::directory_iterator(dirPath))
        {
            std::string filename = entry.path().filename().string();

            // 检查文件名是否匹配模式：focuserPicture_数字.bin
            if (fs::is_symlink(entry.path()) &&
                filename.find("focuserPicture_") == 0 &&
                filename.rfind(".bin") == filename.length() - 4)
            {

                // 查看文件名中间部分是否为数字
                std::string numPart = filename.substr(15, filename.length() - 19); // 去掉"focuserPicture_"和".bin"
                bool isNumeric = !numPart.empty() &&
                                 std::find_if(numPart.begin(), numPart.end(),
                                              [](unsigned char c)
                                              { return !std::isdigit(c); }) == numPart.end();

                if (isNumeric)
                {
                    fs::remove(entry.path());
                    Logger::Log("删除链接文件: " + filename, LogLevel::DEBUG, DeviceType::FOCUSER);
                }
            }
        }
        Logger::Log("所有focuserPicture链接文件已删除", LogLevel::INFO, DeviceType::FOCUSER);
    }
    catch (const std::exception &e)
    {
        Logger::Log("删除链接文件时出错: " + std::string(e.what()), LogLevel::ERROR, DeviceType::FOCUSER);
    }

    std::string Command = "ln -sf " + filePath + " " + vueImagePath + fileName;
    system(Command.c_str());
    Logger::Log("Symbolic link created for new image file.", LogLevel::DEBUG, DeviceType::FOCUSER);

    if (saved)
    {
        // 若有挂起的 ROI，则先应用到 roiAndFocuserInfo（仅更新 ROI_x/ROI_y），使 SaveJpgSuccess 携带的新 ROI
        emit wsThread->sendMessageToClient("SaveJpgSuccess:" + QString::fromStdString(fileName) + ":" + QString::number(roiAndFocuserInfo["ROI_x"]) + ":" + QString::number(roiAndFocuserInfo["ROI_y"]));
        if (hasPendingRoiUpdate)
        {
            hasPendingRoiUpdate = false;
            int boxSideToSend = BoxSideLength;
            if (roiAndFocuserInfo.count("BoxSideLength"))
                boxSideToSend = static_cast<int>(std::lround(roiAndFocuserInfo["BoxSideLength"]));
            const int maxX = std::max(0, glMainCCDSizeX - boxSideToSend);
            const int maxY = std::max(0, glMainCCDSizeY - boxSideToSend);
            int applyX = std::min(std::max(0, pendingRoiX), maxX);
            int applyY = std::min(std::max(0, pendingRoiY), maxY);
            if (applyX % 2 != 0) applyX += (applyX < maxX ? 1 : -1);
            if (applyY % 2 != 0) applyY += (applyY < maxY ? 1 : -1);
            applyX = std::min(std::max(0, applyX), maxX);
            applyY = std::min(std::max(0, applyY), maxY);
            roiAndFocuserInfo["ROI_x"] = int(applyX/glMainCameraBinning);
            roiAndFocuserInfo["ROI_y"] = int(applyY/glMainCameraBinning);
        }

        Logger::Log("SaveJpgSuccess:" + fileName + " to " + filePath + ",image size:" + std::to_string(image16.cols) + "x" + std::to_string(image16.rows), LogLevel::DEBUG, DeviceType::FOCUSER);
        
    }
    else
    {
        Logger::Log("Failed to save image.", LogLevel::ERROR, DeviceType::GUIDER);
    }
    // 释放最终的图像内存
    if (isAutoFocus)
    {
        autoFocus->setCaptureComplete(filename);
    }
    image16.release();
    focusLoopShooting(isFocusLoopShooting); 

}

QPointF MainWindow::selectStar(QList<FITSImage::Star> stars){
    // 1) 边界与输入检查
    if (stars.size() <= 0) {
        Logger::Log("selectStar | no stars", LogLevel::INFO, DeviceType::FOCUSER);
        roiAndFocuserInfo["SelectStarHFR"] = 0.0;
        return QPointF(CurrentPosition, 0);
    }

    // 2) 读取 ROI 与选择点（全图坐标）
    const double boxSide = roiAndFocuserInfo.count("BoxSideLength") ? roiAndFocuserInfo["BoxSideLength"] : BoxSideLength;
    const double roi_x    = roiAndFocuserInfo.count("ROI_x") ? roiAndFocuserInfo["ROI_x"]*glMainCameraBinning : 0;
    const double roi_y    = roiAndFocuserInfo.count("ROI_y") ? roiAndFocuserInfo["ROI_y"]*glMainCameraBinning : 0;
    const double selXFull = roiAndFocuserInfo.count("SelectStarX") ? roiAndFocuserInfo["SelectStarX"] : -1;
    const double selYFull = roiAndFocuserInfo.count("SelectStarY") ? roiAndFocuserInfo["SelectStarY"] : -1;

    // 3) 若已锁定目标星，则优先在本帧中追踪最近的那颗
    const int edgeMargin = 5;
    if (selectedStarLocked && lockedStarFull.x() >= 0 && lockedStarFull.y() >= 0)
    {
        // 防抖：优先在粘滞半径内寻找最近星；否则再全局最近
        int stickIdx = -1; double stickDist2 = std::numeric_limits<double>::max();
        int nearIdx  = -1; double nearDist2  = std::numeric_limits<double>::max();

        for (int i = 0; i < stars.size(); ++i) {
            const auto &s = stars[i];
            if (s.HFR <= 0) continue;

            // 排除 ROI 边缘星点（ROI 相对坐标检查）
            if (s.x <= edgeMargin || s.y <= edgeMargin || 
                s.x >= boxSide - edgeMargin || s.y >= boxSide - edgeMargin) {
                continue;
            }

            // ROI 相对坐标 -> 全图坐标
            const double sxFull = roi_x + s.x;
            const double syFull = roi_y + s.y;
            const double dx = sxFull - lockedStarFull.x();
            const double dy = syFull - lockedStarFull.y();
            const double d2 = dx*dx + dy*dy;

            if (d2 < nearDist2) { nearDist2 = d2; nearIdx = i; }
            if (d2 <= (starStickRadiusPx * starStickRadiusPx) && d2 < stickDist2) {
                stickDist2 = d2; stickIdx = i;
            }
        }

        const int bestIdx = (stickIdx != -1 ? stickIdx : nearIdx);
        if (bestIdx != -1)
        {
            const auto &best = stars[bestIdx];
            const double bestXFull = roi_x + best.x;
            const double bestYFull = roi_y + best.y;
            // 存储 ROI 相对坐标
            roiAndFocuserInfo["SelectStarX"] = best.x;
            roiAndFocuserInfo["SelectStarY"] = best.y;
            roiAndFocuserInfo["SelectStarHFR"] = best.HFR;
            // 更新锁定星点的全图坐标
            lockedStarFull = QPointF(bestXFull, bestYFull);
            Logger::Log("selectStar | tracking locked star: ROI(" + std::to_string(best.x) + "," + std::to_string(best.y) + ") Full(" + std::to_string(bestXFull) + "," + std::to_string(bestYFull) + ") HFR=" + std::to_string(best.HFR), LogLevel::DEBUG, DeviceType::FOCUSER);
            // 判断是否需要居中（挂起到下一帧应用）
            const double centerX = roi_x + boxSide / 2.0;
            const double centerY = roi_y + boxSide / 2.0;
            const double halfWin = boxSide * trackWindowRatio; // 半窗
            const bool outOfWindow = std::abs(bestXFull - centerX) > halfWin || std::abs(bestYFull - centerY) > halfWin;
            if (enableAutoRoiCentering)
            {
                if (outOfWindow) {
                    outOfWindowFrames++;
                } else {
                    outOfWindowFrames = 0;
                }
                if (outOfWindowFrames >= requiredOutFramesForRecentre) {
                    double newRoiX = bestXFull - boxSide / 2.0;
                    double newRoiY = bestYFull - boxSide / 2.0;
                    const int maxX = std::max(0, glMainCCDSizeX - static_cast<int>(boxSide));
                    const int maxY = std::max(0, glMainCCDSizeY - static_cast<int>(boxSide));
                    newRoiX = std::min<double>(std::max<double>(0, newRoiX), maxX);
                    newRoiY = std::min<double>(std::max<double>(0, newRoiY), maxY);
                    int newRoiXi = static_cast<int>(std::lround(newRoiX));
                    int newRoiYi = static_cast<int>(std::lround(newRoiY));
                    if (newRoiXi % 2 != 0) newRoiXi += 1;
                    if (newRoiYi % 2 != 0) newRoiYi += 1;
                    newRoiXi = std::min(std::max(0, newRoiXi), maxX);
                    newRoiYi = std::min(std::max(0, newRoiYi), maxY);
                    // 不立即应用到本帧，挂起到下一帧再更新
                    hasPendingRoiUpdate = true;
                    pendingRoiX = newRoiXi;
                    pendingRoiY = newRoiYi;
                    outOfWindowFrames = 0;
                    Logger::Log("selectStar | tracking window exceeded for consecutive frames, pending ROI recenter", LogLevel::INFO, DeviceType::FOCUSER);
                }
            }
            return lockedStarFull;
        }
        // 若锁定丢失，则继续按下面的自动选择逻辑
        Logger::Log("selectStar | locked star lost, attempting re-selection", LogLevel::WARNING, DeviceType::FOCUSER);
        selectedStarLocked = false;
        lockedStarFull = QPointF(-1, -1);
    }

    // 3) 自动选择 ROI 内"最亮且最大的"星点
    // 简化：按亮度/面积代理：优先 HFR 大、或亮度（这里可用 HFR 或自带的 brightness 字段，如无则用 HFR）
    int bestIdx = -1; double bestScore = -1;
    for (int i = 0; i < stars.size(); ++i) {
        const auto &s = stars[i];
        if (s.HFR <= 0) continue;
        // 排除 ROI 边缘星点（使用 <= 和 >= 确保边缘像素被排除）
        if (s.x <= edgeMargin || s.y <= edgeMargin || 
            s.x >= boxSide - edgeMargin || s.y >= boxSide - edgeMargin) {
            continue;
        }
        // 评分：HFR 越大越好，可叠加亮度，如 s.peak 或 s.flux（若结构里没有则仅用 HFR）
        double score = s.HFR; // TODO: 若有亮度字段可加权
        if (score > bestScore) { bestScore = score; bestIdx = i; }
    }
    if (bestIdx == -1) {
        Logger::Log("selectStar | no valid ROI star for auto-select", LogLevel::WARNING, DeviceType::FOCUSER);
        return QPointF(CurrentPosition, 0);
    }

    const auto &autoBest = stars[bestIdx];
    const double bestXFullAuto = roi_x + autoBest.x;
    const double bestYFullAuto = roi_y + autoBest.y;
    // 存储 ROI 相对坐标
    roiAndFocuserInfo["SelectStarX"] = autoBest.x;
    roiAndFocuserInfo["SelectStarY"] = autoBest.y;
    roiAndFocuserInfo["SelectStarHFR"] = autoBest.HFR;
    // 锁定星点的全图坐标
    lockedStarFull = QPointF(bestXFullAuto, bestYFullAuto);
    selectedStarLocked = true; // 锁定
    Logger::Log("selectStar | auto-selected and locked new star ROI(x,y,HFR)=(" + std::to_string(autoBest.x) + "," + std::to_string(autoBest.y) + "," + std::to_string(autoBest.HFR) + ") Full(" + std::to_string(bestXFullAuto) + "," + std::to_string(bestYFullAuto) + ")", LogLevel::INFO, DeviceType::FOCUSER);
    return lockedStarFull;

    // 旧分支与重复逻辑清理完毕
}

void MainWindow::startAutoFocus()
{
    if (dpFocuser == NULL || dpMainCamera == NULL)
    {
        Logger::Log("AutoFocus | 调焦器或相机未连接", LogLevel::WARNING, DeviceType::FOCUSER);
        isAutoFocus = false;
        emit wsThread->sendMessageToClient("AutoFocusOver:false");
        return;
    }
    // 预处理：统一清理自动对焦相关定时器与信号连接，避免残留或重复
    cleanupAutoFocusConnections();



    if (autoFocus == nullptr)
    {
        autoFocus = new AutoFocus(indi_Client, dpFocuser, dpMainCamera, wsThread,this);
    }
    else
    {
        // 停止旧对象并清理信号连接
        autoFocus->stopAutoFocus();
        cleanupAutoFocusConnections();
        autoFocus->deleteLater();
        autoFocus = nullptr;
        autoFocus = new AutoFocus(indi_Client, dpFocuser, dpMainCamera, wsThread,this);
    }
    autoFocus->setFocuserMinPosition(focuserMinPosition);
    autoFocus->setFocuserMaxPosition(focuserMaxPosition);
    autoFocus->setCoarseDivisionCount(autoFocusCoarseDivisions);
    autoFocus->setDefaultExposureTime(autoFocusExposureTime); // 自动对焦曝光时间（仅作用于自动对焦）
    autoFocus->setUseVirtualData(false);      // 使用虚拟数据
    
    // 设置空程补偿
    if (autofocusBacklashCompensation > 0) {
        autoFocus->setBacklashCompensation(autofocusBacklashCompensation, autofocusBacklashCompensation);
        autoFocus->setUseBacklashCompensation(true);
        Logger::Log(QString("设置自动对焦空程补偿: %1步").arg(autofocusBacklashCompensation).toStdString(), LogLevel::INFO, DeviceType::FOCUSER);
    } else {
        autoFocus->setUseBacklashCompensation(false);
        Logger::Log("自动对焦不使用空程补偿", LogLevel::INFO, DeviceType::FOCUSER);
    }
   for (int i = 1; i <= 11; i++) {
    std::string filename = "~/test_fits/coarse/" + std::to_string(i) + ".fits";
    autoFocus->setCaptureComplete(filename.c_str());
    }
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::roiInfoChanged, this, [this](const QRect &roi)
            {
        if (roi.width() == 0 && roi.height() == 0){
            roiAndFocuserInfo["ROI_x"] = 0;
            roiAndFocuserInfo["ROI_y"] = 0;
            roiAndFocuserInfo["BoxSideLength"] = 300;
            autoFocuserIsROI = false;
        }else{
            roiAndFocuserInfo["ROI_x"] = roi.x();
            roiAndFocuserInfo["ROI_y"] = roi.y();
            roiAndFocuserInfo["BoxSideLength"] = roi.width();
            autoFocuserIsROI = true;
        } }));

    // 连接二次拟合结果信号
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::focusFitUpdated, this, [this](double a, double b, double c, double bestPosition, double minFWHM)
            {
        Logger::Log(QString("接收到focusFitUpdated信号: a=%1, b=%2, c=%3, bestPosition=%4, minFWHM=%5")
                   .arg(a).arg(b).arg(c).arg(bestPosition).arg(minFWHM).toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
        
        // 发送二次曲线数据到前端
        QString curveData = QString("fitQuadraticCurve:%1:%2:%3:%4:%5")
                           .arg(a, 0, 'g', 15)  // 使用科学计数法，保留15位有效数字，确保小系数不被截断
                           .arg(b, 0, 'g', 15)
                           .arg(c, 0, 'g', 15)
                           .arg(bestPosition, 0, 'f', 2)
                           .arg(minFWHM, 0, 'f', 3);
        
        Logger::Log(QString("发送二次曲线数据: %1").arg(curveData).toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(curveData);
        
        // 发送最佳位置点数据
        QString minPointData = QString("fitQuadraticCurve_minPoint:%1:%2")
                              .arg(bestPosition, 0, 'f', 2)
                              .arg(minFWHM, 0, 'f', 3);
        
        Logger::Log(QString("发送最小点数据: %1").arg(minPointData).toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(minPointData);
        
        Logger::Log(QString("二次拟合结果发送完成").toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
    }));

    // 连接数据点信号
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::focusDataPointReady, this, [this](int position, double fwhm, const QString &stage)
            {
        Logger::Log(QString("接收到数据点: position=%1, fwhm=%2, stage=%3")
                   .arg(position).arg(fwhm).arg(stage).toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
        
        // 发送数据点到前端
        QString dataPointMessage = QString("FocusMoveDone:%1:%2")
                                 .arg(position).arg(fwhm);
        
        Logger::Log(QString("发送数据点: %1").arg(dataPointMessage).toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(dataPointMessage);
    }));

    // 连接启动位置更新定时器信号
    // connect(autoFocus, &AutoFocus::startPositionUpdateTimer, this, [this]()
    //         {
    //     Logger::Log("启动位置更新定时器", LogLevel::INFO, DeviceType::FOCUSER);
    //     if (focusMoveTimer) {
    //         focusMoveTimer->start(50); // 改为50毫秒间隔，与实时位置更新保持一致
    //     }
    //     // 确保实时位置更新定时器也在运行
    //     if (realtimePositionTimer) {
    //         realtimePositionTimer->start(50);
    //     }
    // });

    // 连接自动对焦失败信号
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::autofocusFailed, this, [this]()
            {
        Logger::Log("自动对焦失败，发送提示消息到前端", LogLevel::ERROR, DeviceType::FOCUSER);
        
        // 检查是否是由计划任务表触发的自动对焦
        bool wasScheduleTriggered = isScheduleTriggeredAutoFocus;
        int savedCFWpos = schedule_CFWpos; // 保存当前任务的滤镜位置
        
        isAutoFocus = false;
        // 如果是计划触发的自动对焦，向前端报告步骤完成（失败）
        if (wasScheduleTriggered)
        {
            emit wsThread->sendMessageToClient(
                "ScheduleStepState:" +
                QString::number(schedule_currentNum) + ":" +
                "autofocus:" +
                "0:" +
                "0:" +
                "100");
        }
        isScheduleTriggeredAutoFocus = false; // 重置标志
        
        emit wsThread->sendMessageToClient("FitResult:Failed:拟合结果为水平线，未找到最佳焦点");
        
        // 如果是由计划任务表触发的自动对焦失败，仍然继续执行拍摄任务
        if (wasScheduleTriggered)
        {
            Logger::Log("计划任务表触发的自动对焦失败，但仍继续执行拍摄任务", LogLevel::WARNING, DeviceType::MAIN);
            qDebug() << "Schedule-triggered autofocus failed, but continuing with CFW setup...";
            
            // 检查是否已停止计划任务表
            if (StopSchedule)
            {
                Logger::Log("计划任务表已停止，不再继续执行拍摄任务", LogLevel::INFO, DeviceType::MAIN);
                StopSchedule = false;
                return;
            }
            
            // 继续执行设置滤镜和拍摄
            startSetCFW(savedCFWpos);
        }
    }));

    // 连接星点识别结果信号
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::starDetectionResult, this, [this](bool detected, double fwhm)
            {
        if (detected) {
            Logger::Log(QString("识别到星点，FWHM为: %1").arg(fwhm).toStdString(), LogLevel::INFO, DeviceType::FOCUSER);
            emit wsThread->sendMessageToClient(QString("StarDetectionResult:true:%1").arg(fwhm));
        } else {
            Logger::Log("未识别到星点", LogLevel::INFO, DeviceType::FOCUSER);
            emit wsThread->sendMessageToClient("StarDetectionResult:false:0");
        }
    }));

    // 连接自动对焦模式变化信号
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::autoFocusModeChanged, this, [this](const QString &mode, double fwhm)
            {
        Logger::Log(QString("自动对焦模式变化: %1, FWHM: %2").arg(mode).arg(fwhm).toStdString(), LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(QString("AutoFocusModeChanged:%1:%2").arg(mode).arg(fwhm));
    }));

    // 连接自动对焦步骤变化信号 - [AUTO_FOCUS_UI_ENHANCEMENT]
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::autoFocusStepChanged, this, [this](int step, const QString &description)
            {
        Logger::Log(QString("自动对焦步骤变化: 步骤%1 - %2").arg(step).arg(description).toStdString(), LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(QString("AutoFocusStepChanged:%1:%2").arg(step).arg(description));
    }));

  // 连接自动对焦拍摄进度信号：将各阶段拍摄进度转发到前端
  autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::captureProgressChanged,
                                         this, [this](const QString &stage, int current, int total)
                                         {
    Logger::Log(QString("自动对焦拍摄进度: 阶段=%1, 当前=%2, 总数=%3")
                .arg(stage).arg(current).arg(total).toStdString(),
                LogLevel::INFO, DeviceType::FOCUSER);
    emit wsThread->sendMessageToClient(
          QString("AutoFocusCaptureProgress:%1:%2:%3").arg(stage).arg(current).arg(total));
  }));

    // 连接自动对焦完成信号
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::autoFocusCompleted, this, [this](bool success, double bestPosition, double minHFR)
            {
        Logger::Log(QString("自动对焦完成: success=%1, bestPosition=%2, minHFR=%3")
                   .arg(success).arg(bestPosition).arg(minHFR).toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
        
        // 检查是否是由计划任务表触发的自动对焦
        bool wasScheduleTriggered = isScheduleTriggeredAutoFocus;
        int savedCFWpos = schedule_CFWpos; // 保存当前任务的滤镜位置
        
        // 结束阶段：统一清理自动对焦相关定时器与信号连接
        cleanupAutoFocusConnections();
        
        // 如果是计划触发的自动对焦，向前端报告步骤完成
        if (wasScheduleTriggered)
        {
            emit wsThread->sendMessageToClient(
                "ScheduleStepState:" +
                QString::number(schedule_currentNum) + ":" +
                "autofocus:" +
                "0:" +
                "0:" +
                "100");
        }
        
        // 重置计划任务表触发标志
        isScheduleTriggeredAutoFocus = false;
        
        // 确保实时位置更新定时器继续运行
        // if (realtimePositionTimer && !realtimePositionTimer->isActive()) {
        //     realtimePositionTimer->start(50);
        //     Logger::Log("恢复实时位置更新定时器", LogLevel::INFO, DeviceType::FOCUSER);
        // }
        
        // 发送自动对焦完成消息到前端
        QString completeMessage = QString("AutoFocusOver:%1:%2:%3")
                                .arg(success ? "true" : "false")
                                .arg(bestPosition, 0, 'f', 2)
                                .arg(minHFR, 0, 'f', 3);
        
        Logger::Log(QString("发送自动对焦完成消息: %1").arg(completeMessage).toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(completeMessage);
        isAutoFocus = false;
        
        // 发送自动对焦结束事件到前端 - [AUTO_FOCUS_UI_ENHANCEMENT]
        emit wsThread->sendMessageToClient("AutoFocusEnded:自动对焦已结束");
        
        // 如果是由计划任务表触发的自动对焦，完成后继续执行拍摄任务
        if (wasScheduleTriggered)
        {
            Logger::Log("计划任务表触发的自动对焦已完成，继续执行拍摄任务", LogLevel::INFO, DeviceType::MAIN);
            qDebug() << "Schedule-triggered autofocus completed, continuing with CFW setup...";
            
            // 检查是否已停止计划任务表
            if (StopSchedule)
            {
                Logger::Log("计划任务表已停止，不再继续执行拍摄任务", LogLevel::INFO, DeviceType::MAIN);
                StopSchedule = false;
                return;
            }
            
            // 继续执行设置滤镜和拍摄
            startSetCFW(savedCFWpos);
        }
    }));

    autoFocus->startAutoFocus();
    isAutoFocus = true;
    autoFocusStep = 0;
}

void MainWindow::startAutoFocusFineHFROnly()
{
    if (dpFocuser == NULL || dpMainCamera == NULL)
    {
        Logger::Log("AutoFocus (fine-HFR only) | 调焦器或相机未连接", LogLevel::WARNING, DeviceType::FOCUSER);
        isAutoFocus = false;
        emit wsThread->sendMessageToClient("AutoFocusOver:false");
        return;
    }

    // 预处理：统一清理自动对焦相关定时器与信号连接，避免残留或重复
    cleanupAutoFocusConnections();

    if (autoFocus == nullptr)
    {
        autoFocus = new AutoFocus(indi_Client, dpFocuser, dpMainCamera, wsThread,this);
    }
    else
    {
        // 停止旧对象并清理信号连接
        autoFocus->stopAutoFocus();
        cleanupAutoFocusConnections();
        autoFocus->deleteLater();
        autoFocus = nullptr;
        autoFocus = new AutoFocus(indi_Client, dpFocuser, dpMainCamera, wsThread,this);
    }

    // 与常规自动对焦保持一致的参数配置
    autoFocus->setFocuserMinPosition(focuserMinPosition);
    autoFocus->setFocuserMaxPosition(focuserMaxPosition);
    autoFocus->setCoarseDivisionCount(autoFocusCoarseDivisions);
    autoFocus->setDefaultExposureTime(autoFocusExposureTime); // 自动对焦曝光时间（仅作用于自动对焦）
    autoFocus->setUseVirtualData(false);      // 使用实时数据

    // 设置空程补偿
    if (autofocusBacklashCompensation > 0) {
        autoFocus->setBacklashCompensation(autofocusBacklashCompensation, autofocusBacklashCompensation);
        autoFocus->setUseBacklashCompensation(true);
        Logger::Log(QString("设置自动对焦空程补偿: %1步").arg(autofocusBacklashCompensation).toStdString(), LogLevel::INFO, DeviceType::FOCUSER);
    } else {
        autoFocus->setUseBacklashCompensation(false);
        Logger::Log("自动对焦不使用空程补偿", LogLevel::INFO, DeviceType::FOCUSER);
    }

    // 复用与 startAutoFocus / startAutoFocusSuperFineOnly 相同的信号连接，确保前端表现一致
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::roiInfoChanged, this, [this](const QRect &roi)
            {
        if (roi.width() == 0 && roi.height() == 0){
            roiAndFocuserInfo["ROI_x"] = 0;
            roiAndFocuserInfo["ROI_y"] = 0;
            roiAndFocuserInfo["BoxSideLength"] = 300;
            autoFocuserIsROI = false;
        }else{
            roiAndFocuserInfo["ROI_x"] = roi.x();
            roiAndFocuserInfo["ROI_y"] = roi.y();
            roiAndFocuserInfo["BoxSideLength"] = roi.width();
            autoFocuserIsROI = true;
        } }));

    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::focusFitUpdated, this, [this](double a, double b, double c, double bestPosition, double minFWHM)
            {
        Logger::Log(QString("接收到focusFitUpdated信号: a=%1, b=%2, c=%3, bestPosition=%4, minFWHM=%5")
                   .arg(a).arg(b).arg(c).arg(bestPosition).arg(minFWHM).toStdString(),
                   LogLevel::INFO, DeviceType::FOCUSER);

        QString curveData = QString("fitQuadraticCurve:%1:%2:%3:%4:%5")
                           .arg(a, 0, 'g', 15)
                           .arg(b, 0, 'g', 15)
                           .arg(c, 0, 'g', 15)
                           .arg(bestPosition, 0, 'f', 2)
                           .arg(minFWHM, 0, 'f', 3);

        Logger::Log(QString("发送二次曲线数据: %1").arg(curveData).toStdString(),
                   LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(curveData);

        QString minPointData = QString("fitQuadraticCurve_minPoint:%1:%2")
                              .arg(bestPosition, 0, 'f', 2)
                              .arg(minFWHM, 0, 'f', 3);

        Logger::Log(QString("发送最小点数据: %1").arg(minPointData).toStdString(),
                   LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(minPointData);

        Logger::Log(QString("二次拟合结果发送完成").toStdString(),
                   LogLevel::INFO, DeviceType::FOCUSER);
    }));

    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::focusDataPointReady, this, [this](int position, double fwhm, const QString &stage)
            {
        Logger::Log(QString("接收到数据点: position=%1, fwhm=%2, stage=%3")
                   .arg(position).arg(fwhm).arg(stage).toStdString(),
                   LogLevel::INFO, DeviceType::FOCUSER);

        QString dataPointMessage = QString("FocusMoveDone:%1:%2")
                                 .arg(position).arg(fwhm);

        Logger::Log(QString("发送数据点: %1").arg(dataPointMessage).toStdString(),
                   LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(dataPointMessage);
    }));

    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::autofocusFailed, this, [this]()
            {
        Logger::Log("自动对焦失败，发送提示消息到前端", LogLevel::ERROR, DeviceType::FOCUSER);

        bool wasScheduleTriggered = isScheduleTriggeredAutoFocus;
        int savedCFWpos = schedule_CFWpos;

        isAutoFocus = false;
        isScheduleTriggeredAutoFocus = false;

        emit wsThread->sendMessageToClient("FitResult:Failed:拟合结果为水平线，未找到最佳焦点");

        if (wasScheduleTriggered)
        {
            Logger::Log("计划任务表触发的自动对焦失败，但仍继续执行拍摄任务", LogLevel::WARNING, DeviceType::MAIN);
            qDebug() << "Schedule-triggered autofocus failed, but continuing with CFW setup...";

            if (StopSchedule)
            {
                Logger::Log("计划任务表已停止，不再继续执行拍摄任务", LogLevel::INFO, DeviceType::MAIN);
                StopSchedule = false;
                return;
            }

            startSetCFW(savedCFWpos);
        }
    }));

    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::starDetectionResult, this, [this](bool detected, double fwhm)
            {
        if (detected) {
            Logger::Log(QString("识别到星点，FWHM为: %1").arg(fwhm).toStdString(), LogLevel::INFO, DeviceType::FOCUSER);
            emit wsThread->sendMessageToClient(QString("StarDetectionResult:true:%1").arg(fwhm));
        } else {
            Logger::Log("未识别到星点", LogLevel::INFO, DeviceType::FOCUSER);
            emit wsThread->sendMessageToClient("StarDetectionResult:false:0");
        }
    }));

    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::autoFocusModeChanged, this, [this](const QString &mode, double fwhm)
            {
        Logger::Log(QString("自动对焦模式变化: %1, FWHM: %2").arg(mode).arg(fwhm).toStdString(), LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(QString("AutoFocusModeChanged:%1:%2").arg(mode).arg(fwhm));
    }));

    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::autoFocusStepChanged, this, [this](int step, const QString &description)
            {
        Logger::Log(QString("自动对焦步骤变化: 步骤%1 - %2").arg(step).arg(description).toStdString(), LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(QString("AutoFocusStepChanged:%1:%2").arg(step).arg(description));
    }));

    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::autoFocusCompleted, this, [this](bool success, double bestPosition, double minHFR)
            {
        Logger::Log(QString("自动对焦完成: success=%1, bestPosition=%2, minHFR=%3")
                   .arg(success).arg(bestPosition).arg(minHFR).toStdString(),
                   LogLevel::INFO, DeviceType::FOCUSER);

        bool wasScheduleTriggered = isScheduleTriggeredAutoFocus;
        int savedCFWpos = schedule_CFWpos;

        cleanupAutoFocusConnections();

        isScheduleTriggeredAutoFocus = false;

        QString completeMessage = QString("AutoFocusOver:%1:%2:%3")
                                .arg(success ? "true" : "false")
                                .arg(bestPosition, 0, 'f', 2)
                                .arg(minHFR, 0, 'f', 3);

        Logger::Log(QString("发送自动对焦完成消息: %1").arg(completeMessage).toStdString(),
                   LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(completeMessage);
        isAutoFocus = false;

        emit wsThread->sendMessageToClient("AutoFocusEnded:自动对焦已结束");

        if (wasScheduleTriggered)
        {
            Logger::Log("计划任务表触发的自动对焦已完成，继续执行拍摄任务", LogLevel::INFO, DeviceType::MAIN);
            qDebug() << "Schedule-triggered autofocus completed, continuing with CFW setup...";

            if (StopSchedule)
            {
                Logger::Log("计划任务表已停止，不再继续执行拍摄任务", LogLevel::INFO, DeviceType::MAIN);
                StopSchedule = false;
                return;
            }

            startSetCFW(savedCFWpos);
        }
    }));

    // 仅精调(Fine)：从当前位置直接进入 super-fine 精调流程
    autoFocus->startSuperFineFromCurrentPosition();
    isAutoFocus = true;
    autoFocusStep = 0;
}
void MainWindow::startAutoFocusSuperFineOnly()
{
    if (dpFocuser == NULL || dpMainCamera == NULL)
    {
        Logger::Log("AutoFocus (super-fine only) | 调焦器或相机未连接", LogLevel::WARNING, DeviceType::FOCUSER);
        isAutoFocus = false;
        emit wsThread->sendMessageToClient("AutoFocusOver:false");
        return;
    }

    // 预处理：统一清理自动对焦相关定时器与信号连接，避免残留或重复
    cleanupAutoFocusConnections();

    if (autoFocus == nullptr)
    {
        autoFocus = new AutoFocus(indi_Client, dpFocuser, dpMainCamera, wsThread,this);
    }
    else
    {
        // 停止旧对象并清理信号连接
        autoFocus->stopAutoFocus();
        cleanupAutoFocusConnections();
        autoFocus->deleteLater();
        autoFocus = nullptr;
        autoFocus = new AutoFocus(indi_Client, dpFocuser, dpMainCamera, wsThread,this);
    }

    // 与常规自动对焦保持一致的参数配置
    autoFocus->setFocuserMinPosition(focuserMinPosition);
    autoFocus->setFocuserMaxPosition(focuserMaxPosition);
    autoFocus->setCoarseDivisionCount(autoFocusCoarseDivisions);
    autoFocus->setDefaultExposureTime(autoFocusExposureTime); // 自动对焦曝光时间（仅作用于自动对焦）
    autoFocus->setUseVirtualData(false);      // 使用虚拟数据

    // 设置空程补偿
    if (autofocusBacklashCompensation > 0) {
        autoFocus->setBacklashCompensation(autofocusBacklashCompensation, autofocusBacklashCompensation);
        autoFocus->setUseBacklashCompensation(true);
        Logger::Log(QString("设置自动对焦空程补偿: %1步").arg(autofocusBacklashCompensation).toStdString(), LogLevel::INFO, DeviceType::FOCUSER);
    } else {
        autoFocus->setUseBacklashCompensation(false);
        Logger::Log("自动对焦不使用空程补偿", LogLevel::INFO, DeviceType::FOCUSER);
    }

    // 复用与 startAutoFocus 相同的信号连接
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::roiInfoChanged, this, [this](const QRect &roi)
            {
        if (roi.width() == 0 && roi.height() == 0){
            roiAndFocuserInfo["ROI_x"] = 0;
            roiAndFocuserInfo["ROI_y"] = 0;
            roiAndFocuserInfo["BoxSideLength"] = 300;
            autoFocuserIsROI = false;
        }else{
            roiAndFocuserInfo["ROI_x"] = roi.x();
            roiAndFocuserInfo["ROI_y"] = roi.y();
            roiAndFocuserInfo["BoxSideLength"] = roi.width();
            autoFocuserIsROI = true;
        } }));

    // 连接二次拟合结果信号
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::focusFitUpdated, this, [this](double a, double b, double c, double bestPosition, double minFWHM)
            {
        Logger::Log(QString("接收到focusFitUpdated信号: a=%1, b=%2, c=%3, bestPosition=%4, minFWHM=%5")
                   .arg(a).arg(b).arg(c).arg(bestPosition).arg(minFWHM).toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
        
        // 发送二次曲线数据到前端
        QString curveData = QString("fitQuadraticCurve:%1:%2:%3:%4:%5")
                           .arg(a, 0, 'g', 15)  // 使用科学计数法，保留15位有效数字，确保小系数不被截断
                           .arg(b, 0, 'g', 15)
                           .arg(c, 0, 'g', 15)
                           .arg(bestPosition, 0, 'f', 2)
                           .arg(minFWHM, 0, 'f', 3);
        
        Logger::Log(QString("发送二次曲线数据: %1").arg(curveData).toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(curveData);
        
        // 发送最佳位置点数据
        QString minPointData = QString("fitQuadraticCurve_minPoint:%1:%2")
                              .arg(bestPosition, 0, 'f', 2)
                              .arg(minFWHM, 0, 'f', 3);
        
        Logger::Log(QString("发送最小点数据: %1").arg(minPointData).toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(minPointData);
        
        Logger::Log(QString("二次拟合结果发送完成").toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
    }));

    // 连接数据点信号
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::focusDataPointReady, this, [this](int position, double fwhm, const QString &stage)
            {
        Logger::Log(QString("接收到数据点: position=%1, fwhm=%2, stage=%3")
                   .arg(position).arg(fwhm).arg(stage).toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
        
        // 发送数据点到前端
        QString dataPointMessage = QString("FocusMoveDone:%1:%2")
                                 .arg(position).arg(fwhm);
        
        Logger::Log(QString("发送数据点: %1").arg(dataPointMessage).toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(dataPointMessage);
    }));

    // 连接自动对焦失败信号
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::autofocusFailed, this, [this]()
            {
        Logger::Log("自动对焦失败，发送提示消息到前端", LogLevel::ERROR, DeviceType::FOCUSER);
        
        // 检查是否是由计划任务表触发的自动对焦
        bool wasScheduleTriggered = isScheduleTriggeredAutoFocus;
        int savedCFWpos = schedule_CFWpos; // 保存当前任务的滤镜位置
        
        isAutoFocus = false;
        isScheduleTriggeredAutoFocus = false; // 重置标志
        
        emit wsThread->sendMessageToClient("FitResult:Failed:拟合结果为水平线，未找到最佳焦点");
        
        // 如果是由计划任务表触发的自动对焦失败，仍然继续执行拍摄任务
        if (wasScheduleTriggered)
        {
            Logger::Log("计划任务表触发的自动对焦失败，但仍继续执行拍摄任务", LogLevel::WARNING, DeviceType::MAIN);
            qDebug() << "Schedule-triggered autofocus failed, but continuing with CFW setup...";
            
            // 检查是否已停止计划任务表
            if (StopSchedule)
            {
                Logger::Log("计划任务表已停止，不再继续执行拍摄任务", LogLevel::INFO, DeviceType::MAIN);
                StopSchedule = false;
                return;
            }
            
            // 继续执行设置滤镜和拍摄
            startSetCFW(savedCFWpos);
        }
    }));

    // 连接星点识别结果信号
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::starDetectionResult, this, [this](bool detected, double fwhm)
            {
        if (detected) {
            Logger::Log(QString("识别到星点，FWHM为: %1").arg(fwhm).toStdString(), LogLevel::INFO, DeviceType::FOCUSER);
            emit wsThread->sendMessageToClient(QString("StarDetectionResult:true:%1").arg(fwhm));
        } else {
            Logger::Log("未识别到星点", LogLevel::INFO, DeviceType::FOCUSER);
            emit wsThread->sendMessageToClient("StarDetectionResult:false:0");
        }
    }));

    // 连接自动对焦模式变化信号
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::autoFocusModeChanged, this, [this](const QString &mode, double fwhm)
            {
        Logger::Log(QString("自动对焦模式变化: %1, FWHM: %2").arg(mode).arg(fwhm).toStdString(), LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(QString("AutoFocusModeChanged:%1:%2").arg(mode).arg(fwhm));
    }));

    // 连接自动对焦步骤变化信号 - [AUTO_FOCUS_UI_ENHANCEMENT]
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::autoFocusStepChanged, this, [this](int step, const QString &description)
            {
        Logger::Log(QString("自动对焦步骤变化: 步骤%1 - %2").arg(step).arg(description).toStdString(), LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(QString("AutoFocusStepChanged:%1:%2").arg(step).arg(description));
    }));

    // 连接自动对焦完成信号
    autoFocusConnections.push_back(connect(autoFocus, &AutoFocus::autoFocusCompleted, this, [this](bool success, double bestPosition, double minHFR)
            {
        Logger::Log(QString("自动对焦完成: success=%1, bestPosition=%2, minHFR=%3")
                   .arg(success).arg(bestPosition).arg(minHFR).toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
        
        // 检查是否是由计划任务表触发的自动对焦
        bool wasScheduleTriggered = isScheduleTriggeredAutoFocus;
        int savedCFWpos = schedule_CFWpos; // 保存当前任务的滤镜位置
        
        // 结束阶段：统一清理自动对焦相关定时器与信号连接
        cleanupAutoFocusConnections();
        
        // 重置计划任务表触发标志
        isScheduleTriggeredAutoFocus = false;
        
        // 发送自动对焦完成消息到前端
        QString completeMessage = QString("AutoFocusOver:%1:%2:%3")
                                .arg(success ? "true" : "false")
                                .arg(bestPosition, 0, 'f', 2)
                                .arg(minHFR, 0, 'f', 3);
        
        Logger::Log(QString("发送自动对焦完成消息: %1").arg(completeMessage).toStdString(), 
                   LogLevel::INFO, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient(completeMessage);
        isAutoFocus = false;
        
        // 发送自动对焦结束事件到前端 - [AUTO_FOCUS_UI_ENHANCEMENT]
        emit wsThread->sendMessageToClient("AutoFocusEnded:自动对焦已结束");
        
        // 如果是由计划任务表触发的自动对焦，完成后继续执行拍摄任务
        if (wasScheduleTriggered)
        {
            Logger::Log("计划任务表触发的自动对焦已完成，继续执行拍摄任务", LogLevel::INFO, DeviceType::MAIN);
            qDebug() << "Schedule-triggered autofocus completed, continuing with CFW setup...";
            
            // 检查是否已停止计划任务表
            if (StopSchedule)
            {
                Logger::Log("计划任务表已停止，不再继续执行拍摄任务", LogLevel::INFO, DeviceType::MAIN);
                StopSchedule = false;
                return;
            }
            
            // 继续执行设置滤镜和拍摄
            startSetCFW(savedCFWpos);
        }
    }));

    autoFocus->startSuperFineFromCurrentPosition();
    isAutoFocus = true;
    autoFocusStep = 0;
}

void MainWindow::startScheduleAutoFocus()
{
    // 检查设备是否连接
    if (dpFocuser == NULL || dpMainCamera == NULL)
    {
        Logger::Log("计划任务表自动对焦 | 调焦器或相机未连接，跳过自动对焦", LogLevel::WARNING, DeviceType::FOCUSER);
        isScheduleTriggeredAutoFocus = false;
        // 如果自动对焦失败，直接继续执行拍摄任务
        startSetCFW(schedule_CFWpos);
        return;
    }
    
    // 如果已经有自动对焦在运行，先停止
    if (isAutoFocus && autoFocus != nullptr)
    {
        Logger::Log("计划任务表自动对焦 | 检测到已有自动对焦在运行，先停止", LogLevel::INFO, DeviceType::FOCUSER);
        autoFocus->stopAutoFocus();
        cleanupAutoFocusConnections();
        autoFocus->deleteLater();
        autoFocus = nullptr;
        isAutoFocus = false;
    }
    
    // 标记这是由计划任务表触发的自动对焦
    isScheduleTriggeredAutoFocus = true;
    // 向前端发送步骤状态：当前行进入自动对焦阶段
    emit wsThread->sendMessageToClient(
        "ScheduleStepState:" +
        QString::number(schedule_currentNum) + ":" +
        "autofocus:" +
        "0:" +
        "0:" +
        "0");
    
    // 调用通用的自动对焦启动函数
    Logger::Log("计划任务表自动对焦 | Refocus=ON：开始执行自动对焦最后一步精调(super-fine)", LogLevel::INFO, DeviceType::MAIN);
    // 仅触发自动对焦的最后一步精调：从当前位置进入 super-fine 精调（跳过粗调/精调）
    startAutoFocusSuperFineOnly();
}

void MainWindow::cleanupAutoFocusConnections()
{
    if (autoFocus) {
        // 逐个断开并清空记录的连接，覆盖所有已登记连接
        for (const QMetaObject::Connection &c : autoFocusConnections) {
            QObject::disconnect(c);
        }
        autoFocusConnections.clear();
        // 保险起见再断开双方所有连接
        disconnect(autoFocus, nullptr, this, nullptr);
        disconnect(this, nullptr, autoFocus, nullptr);
    }
}


void MainWindow::getFocuserLoopingState()
{
    if (isFocusLoopShooting)
    {
        emit wsThread->sendMessageToClient("setFocuserLoopingState:true");
    }
    else
    {
        emit wsThread->sendMessageToClient("setFocuserLoopingState:false");
    }
}

void MainWindow::sendRoiInfo()
{
    Logger::Log("==========================================", LogLevel::INFO, DeviceType::FOCUSER);
    for (auto it = roiAndFocuserInfo.begin(); it != roiAndFocuserInfo.end(); ++it)
    {
        Logger::Log("roiAndFocuserInfo | Key:" + it->first + " Value:" + std::to_string(it->second), LogLevel::INFO, DeviceType::FOCUSER);
    }
    Logger::Log("==========================================", LogLevel::INFO, DeviceType::FOCUSER);
    // 检查并获取参数，如果不存在则使用默认值
    double boxSideLength = roiAndFocuserInfo.count("BoxSideLength") ? roiAndFocuserInfo["BoxSideLength"] : 300;
    double roi_x = roiAndFocuserInfo.count("ROI_x") ? roiAndFocuserInfo["ROI_x"] : 1;
    double roi_y = roiAndFocuserInfo.count("ROI_y") ? roiAndFocuserInfo["ROI_y"] : 1;
    double visibleX = roiAndFocuserInfo.count("VisibleX") ? roiAndFocuserInfo["VisibleX"] : 0;
    double visibleY = roiAndFocuserInfo.count("VisibleY") ? roiAndFocuserInfo["VisibleY"] : 0;
    double scale = roiAndFocuserInfo.count("scale") ? roiAndFocuserInfo["scale"] : 1;
    double selectStarX = roiAndFocuserInfo.count("SelectStarX") ? roiAndFocuserInfo["SelectStarX"] : -1;
    double selectStarY = roiAndFocuserInfo.count("SelectStarY") ? roiAndFocuserInfo["SelectStarY"] : -1;

    Logger::Log("sendRoiInfo | 发送参数 roi_x:" + std::to_string(roi_x) + " roi_y:" + std::to_string(roi_y) + " boxSideLength:" + std::to_string(boxSideLength) + " visibleX:" + std::to_string(visibleX) + " visibleY:" + std::to_string(visibleY) + " scale:" + std::to_string(scale) + " selectStarX:" + std::to_string(selectStarX) + " selectStarY:" + std::to_string(selectStarY), LogLevel::INFO, DeviceType::FOCUSER);

    // 发送参数
    emit wsThread->sendMessageToClient("SetRedBoxState:" + QString::number(boxSideLength) + ":" + QString::number(roi_x) + ":" + QString::number(roi_y));
    emit wsThread->sendMessageToClient("SetVisibleArea:" + QString::number(visibleX) + ":" + QString::number(visibleY) + ":" + QString::number(scale));
    emit wsThread->sendMessageToClient("SetSelectStars:" + QString::number(selectStarX) + ":" + QString::number(selectStarY));
}
void MainWindow::updateCPUInfo()
{
    // 获取CPU温度和使用率
    QProcess process;
    // 在树莓派上，可以通过读取 /sys/class/thermal/thermal_zone0/temp 文件来获取 CPU 温度
    process.start("cat", QStringList() << "/sys/class/thermal/thermal_zone0/temp");
    process.waitForFinished();
    QString output = process.readAllStandardOutput();
    float cpuTemp = output.toFloat() / 1000; // 转换为摄氏度
    if (process.error() != QProcess::UnknownError)
    {
        cpuTemp = std::numeric_limits<float>::quiet_NaN(); // 如果获取失败，设置为 NaN
    }

    // 在树莓派上，可以通过运行 'top' 命令并解析输出来获取 CPU 使用率
    process.start("sh", QStringList() << "-c" << "top -b -n1 | grep 'Cpu(s)' | awk '{print $2}' | cut -c 1-4");
    process.waitForFinished();
    output = process.readAllStandardOutput();
    QStringList cpuUsages = output.split("\n");
    float cpuUsage = 0;
    if (cpuUsages.size() > 0)
    {
        cpuUsage = cpuUsages[0].toDouble();
    }
    if (process.error() != QProcess::UnknownError)
    {
        cpuUsage = std::numeric_limits<float>::quiet_NaN(); // 如果获取失败，设置为 NaN
    }

    // Logger::Log("updateCPUInfo | CPU Temp: " + std::to_string(cpuTemp) + ", CPU Usage: " + std::to_string(cpuUsage), LogLevel::DEBUG, DeviceType::MAIN);
    emit wsThread->sendMessageToClient("updateCPUInfo:" + QString::number(cpuTemp) + ":" + QString::number(cpuUsage));
}

void MainWindow::getMainCameraParameters()
{
    Logger::Log("getMainCameraParameters start ...", LogLevel::DEBUG, DeviceType::MAIN);
    QMap<QString, QString> parameters = Tools::readParameters("MainCamera");
    QString order = "setMainCameraParameters";
    for (auto it = parameters.begin(); it != parameters.end(); ++it)
    {
        Logger::Log("getMainCameraParameters | " + it.key().toStdString() + ":" + it.value().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        if (it.key() == "Save Folder" ) {
            QString oldSaveFolder = it.value();
            // 兼容旧的"default"，转换为"local"
            if (oldSaveFolder == "default") {
                oldSaveFolder = "local";
                it.value() = "local";
            }
            
            if (oldSaveFolder == "local") {
                ImageSaveBaseDirectory = QString::fromStdString(ImageSaveBasePath);
                saveMode = "local";
            } else if (usbMountPointsMap.contains(oldSaveFolder)) {
                ImageSaveBaseDirectory = usbMountPointsMap[oldSaveFolder] + "/QUARCS_ImageSave";
                saveMode = oldSaveFolder;
            } else {
                // U盘不存在，回退到本地
                it.value() = "local";
                ImageSaveBaseDirectory = QString::fromStdString(ImageSaveBasePath);
                saveMode = "local";
                Logger::Log("LoadParameter | USB '" + oldSaveFolder.toStdString() + "' not found, using local", LogLevel::WARNING, DeviceType::MAIN);
            }
        }
        order += ":" + it.key() + ":" + it.value();
        if (it.key() == "RedBoxSize") {
            BoxSideLength = it.value().toInt();
            roiAndFocuserInfo["BoxSideLength"] = BoxSideLength;
        }
        if (it.key() == "ROI_x") roiAndFocuserInfo["ROI_x"] = it.value().toDouble();
        if (it.key() == "ROI_y") roiAndFocuserInfo["ROI_y"] = it.value().toDouble();
        if (it.key() == "AutoSave") {
            mainCameraAutoSave = (it.value() == "true");
            // Logger::Log("/*/*/*/*/*/*getMainCameraParameters | AutoSave: " + std::to_string(mainCameraAutoSave), LogLevel::DEBUG, DeviceType::MAIN);
        }
        if (it.key() == "SaveFailedParse") {
            mainCameraSaveFailedParse = (it.value() == "true");
        }
        if (it.key() == "Temperature") {
            CameraTemperature = it.value().toDouble();
        }
        if (it.key() == "Gain") {
            CameraGain = it.value().toInt();
        }
        if (it.key() == "Offset") {
            ImageOffset = it.value().toDouble();
        }
    }
    Logger::Log("getMainCameraParameters finish!", LogLevel::DEBUG, DeviceType::MAIN);
    emit wsThread->sendMessageToClient(order);
    emit wsThread->sendMessageToClient("MainCameraCFA:" + MainCameraCFA);
}

void MainWindow::getMountParameters()
{
    Logger::Log("getMountUiInfo ...", LogLevel::DEBUG, DeviceType::MAIN);
    QMap<QString, QString> parameters = Tools::readParameters("Mount");
    for (auto it = parameters.begin(); it != parameters.end(); ++it)
    {
        Logger::Log("getMountParameters | " + it.key().toStdString() + ":" + it.value().toStdString(), LogLevel::DEBUG, DeviceType::MAIN);
        if (it.key() == "AutoFlip"){
            emit wsThread->sendMessageToClient("AutoFlip:" + it.value());
            isAutoFlip = it.value() == "true";
            continue;
        }
        if (it.key() == "GotoThenSolve"){
            emit wsThread->sendMessageToClient("GotoThenSolve:" + it.value());
            GotoThenSolve = it.value() == "true";
            continue;
        }
        emit wsThread->sendMessageToClient(it.key() + ":" + it.value());
    }

    // 将当前 Mount 串口列表与已保存串口下发给前端（若无保存则 savedPort 为空）
    sendSerialPortOptions("Mount");

    Logger::Log("getMountParameters finish!", LogLevel::DEBUG, DeviceType::MAIN);
}

void MainWindow::synchronizeTime(QString time, QString date)
{
    Logger::Log("synchronizeTime start ...", LogLevel::DEBUG, DeviceType::MAIN);
    Logger::Log("synchronizeTime time: " + time.toStdString() + ", date: " + date.toStdString(), LogLevel::DEBUG, DeviceType::MAIN);

    // 先禁用自动时间同步
    Logger::Log("Disabling automatic time synchronization...", LogLevel::DEBUG, DeviceType::MAIN);
    
    // 禁用 systemd-timesyncd 服务
    int disableResult1 = system("sudo systemctl stop systemd-timesyncd");
    int disableResult2 = system("sudo systemctl disable systemd-timesyncd");
    
    // 禁用 NTP 同步
    int disableResult3 = system("sudo timedatectl set-ntp false");
    
    if (disableResult1 != 0 || disableResult2 != 0 || disableResult3 != 0)
    {
        Logger::Log("Warning: Failed to disable some automatic time sync services", LogLevel::WARNING, DeviceType::MAIN);
    }
    else
    {
        Logger::Log("Automatic time synchronization disabled successfully", LogLevel::DEBUG, DeviceType::MAIN);
    }

    // 等待一秒确保服务完全停止
    QThread::msleep(1000);

    // Create the command string
    QString command = "sudo date -s \"" + date + " " + time + "\"";

    Logger::Log("synchronizeTime command: " + command.toStdString(), LogLevel::DEBUG, DeviceType::MAIN);

    // Execute the command
    int result = system(command.toStdString().c_str());

    if (result == 0)
    {
        Logger::Log("synchronizeTime finish!", LogLevel::DEBUG, DeviceType::MAIN);
    }
    else
    {
        Logger::Log("synchronizeTime failed!", LogLevel::ERROR, DeviceType::MAIN);
    }
}

void MainWindow::setMountLocation(QString lat, QString lon)
{
    Logger::Log("setMountLocation start ...", LogLevel::DEBUG, DeviceType::MAIN);
    observatorylatitude = lat.toDouble();
    observatorylongitude = lon.toDouble();
    if (dpMount != nullptr)
    {
        indi_Client->mountState.updateHomeRAHours(observatorylatitude, observatorylongitude);
        indi_Client->setLocation(dpMount, observatorylatitude, observatorylongitude, 50);
 
    }
}

void MainWindow::setMountUTC(QString time, QString date)
{
    Logger::Log("setMountUTC start ...", LogLevel::DEBUG, DeviceType::MAIN);
    if (dpMount == nullptr)
    {
        Logger::Log("setMountUTC | dpMount is nullptr", LogLevel::WARNING, DeviceType::MAIN);
        return;
    }
    QDateTime datetime = QDateTime::fromString(date + "T" + time, Qt::ISODate);
    indi_Client->setTimeUTC(dpMount, datetime);
    Logger::Log("UTC Time set for Mount: " + datetime.toString(Qt::ISODate).toStdString(), LogLevel::INFO, DeviceType::MAIN);
    indi_Client->getTimeUTC(dpMount, datetime);
    Logger::Log("UTC Time: " + datetime.currentDateTimeUtc().toString(Qt::ISODate).toStdString(), LogLevel::INFO, DeviceType::MAIN);
}

void MainWindow::getLastSelectDevice()
{
    SystemDeviceList newSystemdevicelist = Tools::readSystemDeviceList();

    // 检查是否有历史设备配置记录
    bool hasHistoryConfig = false;
    for (int i = 0; i < newSystemdevicelist.system_devices.size(); i++)
    {
        if (!newSystemdevicelist.system_devices[i].DriverIndiName.isEmpty() &&
            !newSystemdevicelist.system_devices[i].Description.isEmpty())
        {
            hasHistoryConfig = true;
            break;
        }
    }

    if (!hasHistoryConfig)
    {
        Logger::Log("No historical connection records found", LogLevel::ERROR, DeviceType::MAIN);
        return;
    }

    // 只有当未连接设备时才触发设备选择
    if (ConnectedDevices.size() == 0)
    {
        systemdevicelist = newSystemdevicelist;
        loadSelectedDriverList();
        Logger::Log("Last Connected Device Has Send", LogLevel::INFO, DeviceType::MAIN);
    }
    else
    {
        Logger::Log("Devices already connected, skip loading last selection", LogLevel::INFO, DeviceType::MAIN);
    }
}

/** 自动极轴校准 */
bool MainWindow::initPolarAlignment()
{
    if (dpMount == nullptr || dpMainCamera == nullptr)
    {
        Logger::Log("initPolarAlignment | dpMount or dpMainCamera is nullptr", LogLevel::ERROR, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("StartAutoPolarAlignmentStatus:false:Failed to start polar alignment,dpMount or dpMainCamera is nullptr");
        return false;
    }
    if (indi_Client == nullptr)
    {
        Logger::Log("initPolarAlignment | indi_Client is nullptr", LogLevel::ERROR, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("StartAutoPolarAlignmentStatus:false:Failed to start polar alignment,indi_Client is nullptr");
        return false;
    }
    if (glFocalLength == 0)
    {
        Logger::Log("initPolarAlignment | glFocalLength is 0, trying to set default value", LogLevel::WARNING, DeviceType::MAIN);
        // 尝试从配置文件读取默认焦距
        QMap<QString, QString> parameters = Tools::readParameters("MainCamera");
        if (parameters.contains("FocalLength"))
        {
            glFocalLength = parameters["FocalLength"].toInt();
            Logger::Log("initPolarAlignment | Loaded focal length from config: " + std::to_string(glFocalLength), LogLevel::INFO, DeviceType::MAIN);
        }
        else
        {
            Logger::Log("initPolarAlignment | focal length is not set, please set it in the config file", LogLevel::WARNING, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("StartAutoPolarAlignmentStatus:false:Failed to start polar alignment,focal length is not set");
            return false;
        }
    }
    if (observatorylatitude == 0 || observatorylongitude == 0)
    {
        Logger::Log("initPolarAlignment | observatorylatitude or observatorylongitude is 0", LogLevel::WARNING, DeviceType::MAIN);
    }

    // 检查相机参数是否有效
    if (glCameraSize_width <= 0 || glCameraSize_height <= 0)
    {
        // 如果相机参数无效，则尝试重新获取
        double pixelsize, pixelsizX, pixelsizY;
        int maxX, maxY, bitDepth;
        indi_Client->getCCDBasicInfo(dpMainCamera, maxX, maxY, pixelsize, pixelsizX, pixelsizY, bitDepth);
        glCameraSize_width = maxX * pixelsize / 1000;
        glCameraSize_width = std::round(glCameraSize_width * 10) / 10;
        glCameraSize_height = maxY * pixelsize / 1000;
        glCameraSize_height = std::round(glCameraSize_height * 10) / 10;
        if (glCameraSize_width <= 0 || glCameraSize_height <= 0)
        {
            Logger::Log("initPolarAlignment | Camera size parameters are invalid", LogLevel::ERROR, DeviceType::MAIN);
            emit wsThread->sendMessageToClient("StartAutoPolarAlignmentStatus:false:Failed to start polar alignment,Camera size parameters are invalid");
            return false;
        }
    }

    // 自动极轴校准初始化
    polarAlignment = new PolarAlignment(indi_Client, dpMount, dpMainCamera);
    if (polarAlignment == nullptr)
    {
        Logger::Log("initPolarAlignment | Failed to create PolarAlignment object", LogLevel::ERROR, DeviceType::MAIN);
        emit wsThread->sendMessageToClient("StartAutoPolarAlignmentStatus:false:Failed to start polar alignment,Failed to create PolarAlignment object");
        return false;
    }

    // 设置配置参数
    PolarAlignmentConfig config;
    config.defaultExposureTime = 1000;         // 默认曝光时间1秒
    config.recoveryExposureTime = 5000;        // 恢复曝光时间5秒
    config.raRotationAngle = 15.0;              // RA轴每次移动1度
    config.decRotationAngle = 15.0;             // DEC轴移动1度
    config.maxRetryAttempts = 3;               // 最大重试3次
    config.captureAndAnalysisTimeout = 30000;  // 拍摄和分析超时30秒
    config.movementTimeout = 15000;            // 移动超时15秒
    config.maxAdjustmentAttempts = 3;          // 最大调整尝试次数3次
    config.adjustmentAngleReduction = 0.5;     // 调整角度减半
    config.cameraWidth = glCameraSize_width;   // 相机传感器宽度（毫米）
    config.cameraHeight = glCameraSize_height; // 相机传感器高度（毫米）
    config.focalLength = glFocalLength;        // 焦距（毫米）
    config.latitude = observatorylatitude;     // 观测地点纬度（度）
    config.longitude = observatorylongitude;   // 观测地点经度（度）
    config.finalVerificationThreshold = 0.5;   // 最终验证精度阈值（度）

    // 设置配置
    polarAlignment->setConfig(config);

    // 连接信号
    connect(polarAlignment, &PolarAlignment::stateChanged,
            [this](PolarAlignmentState state, QString message, int percentage)
            {
                qDebug() << "状态改变:" << static_cast<int>(state) << " 消息:" << message << " 进度:" << percentage;
                emit this->wsThread->sendMessageToClient(QString("PolarAlignmentState:") +
                                                        (polarAlignment->isRunning() ? "true" : "false") + ":" +
                                                        QString::number(static_cast<int>(state)) + ":" +
                                                        message + ":" +
                                                        QString::number(percentage));

                // 极轴校准结束/停止后，自动恢复赤道仪跟踪
                if (state == PolarAlignmentState::IDLE ||
                    state == PolarAlignmentState::COMPLETED ||
                    state == PolarAlignmentState::FAILED ||
                    state == PolarAlignmentState::USER_INTERVENTION)
                {
                    if (indi_Client != nullptr && dpMount != nullptr)
                    {
                        indi_Client->setTelescopeTrackEnable(dpMount, true);

                        bool isTrack = false;
                        indi_Client->getTelescopeTrackEnable(dpMount, isTrack);
                        emit this->wsThread->sendMessageToClient(isTrack ? "TelescopeTrack:ON"
                                                                         : "TelescopeTrack:OFF");

                        Logger::Log("PolarAlignment: Telescope tracking restored after polar alignment",
                                    LogLevel::INFO, DeviceType::MAIN);
                    }
                    emit this->wsThread->sendMessageToClient("PolarAlignmentState:false:0:极轴校准已停止:0");
                }
            });

            connect(polarAlignment, &PolarAlignment::adjustmentGuideData,
                [this](double ra, double dec,
                       double ra0, double dec0, double ra1, double dec1,
                       double ra2, double dec2, double ra3, double dec3,
                       double targetRa, double targetDec,
                       double offsetRa, double offsetDec,
                       const QString &adjustmentRa, const QString &adjustmentDec,
                       double fakePolarRA, double fakePolarDEC,
                       double realPolarRA, double realPolarDEC)
                {
                    QString logMsg = QString("PolarAlignmentAdjustmentGuideData:%1:%2:%3:%4:%5:%6:%7:%8:%9:%10:%11:%12:%13:%14:%15:%16:%17:%18:%19:%20")
                                         .arg(ra)
                                         .arg(dec)
                                         .arg(ra0)
                                         .arg(dec0)
                                         .arg(ra1)
                                         .arg(dec1)
                                         .arg(ra2)
                                         .arg(dec2)
                                         .arg(ra3)
                                         .arg(dec3)
                                         .arg(targetRa)
                                         .arg(targetDec)
                                         .arg(offsetRa)
                                         .arg(offsetDec)
                                         .arg(adjustmentRa)
                                         .arg(adjustmentDec)
                                         .arg(fakePolarRA)
                                         .arg(fakePolarDEC)
                                         .arg(realPolarRA)
                                         .arg(realPolarDEC);
                    Logger::Log("目标点: " + std::to_string(targetRa) + ", " + std::to_string(targetDec), LogLevel::INFO, DeviceType::MAIN);
                    Logger::Log("假极轴: " + std::to_string(fakePolarRA) + ", " + std::to_string(fakePolarDEC), LogLevel::INFO, DeviceType::MAIN);
                    Logger::Log("真极轴: " + std::to_string(realPolarRA) + ", " + std::to_string(realPolarDEC), LogLevel::INFO, DeviceType::MAIN);
                    Logger::Log(logMsg.toStdString(), LogLevel::INFO, DeviceType::MAIN);
                    emit this->wsThread->sendMessageToClient(logMsg);
                });

            // 连接调整阶段进度信号
            connect(polarAlignment, &PolarAlignment::guidanceAdjustmentStepProgress,
                [this](GuidanceAdjustmentStep step, QString message, int starCount)
                {
                    QString logMsg = QString("PolarAlignmentGuidanceStepProgress:%1:%2:%3")
                                         .arg(static_cast<int>(step))
                                         .arg(message)
                                         .arg(starCount);
                    Logger::Log(logMsg.toStdString(), LogLevel::INFO, DeviceType::MAIN);
                    emit this->wsThread->sendMessageToClient(logMsg);
                });

    Logger::Log("initPolarAlignment | PolarAlignment initialized successfully", LogLevel::INFO, DeviceType::MAIN);
    return true;
}

void MainWindow::focusMoveToMin()
{
    if (dpFocuser == nullptr)
    {
        Logger::Log("focusMoveToMin | dpFocuser is nullptr", LogLevel::ERROR, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient("focusMoveFailed:focuser is not connected");
        return;
    }

    int min, max, step, value;
    indi_Client->getFocuserRange(dpFocuser, min, max, step, value); // 获取焦点器范围
    indi_Client->syncFocuserPosition(dpFocuser, (max + min) / 2);   // 同步焦点器位置到中间位置
    emit wsThread->sendMessageToClient("FocusPosition:" + QString::number(CurrentPosition) + ":" + QString::number(CurrentPosition));
    // indi_Client->moveFocuserToAbsolutePosition(dpFocuser, min); // 移动到最小位置
    indi_Client->setFocuserMoveDiretion(dpFocuser, true);
    int steps = std::min(CurrentPosition - min, 60000);
    indi_Client->moveFocuserSteps(dpFocuser, steps);
    TargetPosition = CurrentPosition - steps;
    focusMoveToMaxorMinTimer = new QTimer(this);
    CurrentPosition = FocuserControl_getPosition();
    lastPosition = CurrentPosition;
    connect(focusMoveToMaxorMinTimer, &QTimer::timeout, this, [this, min]()
            {
        CurrentPosition = FocuserControl_getPosition();
        emit wsThread->sendMessageToClient("FocusPosition:" + QString::number(CurrentPosition) + ":" + QString::number(CurrentPosition));
        if (CurrentPosition == TargetPosition){
            int min, max, step, value;
            indi_Client->getFocuserRange(dpFocuser, min, max, step, value); // 获取焦点器范围
            int steps = std::min(CurrentPosition - min,60000);
            indi_Client->moveFocuserSteps(dpFocuser, steps);
            TargetPosition = CurrentPosition-steps;
            emit wsThread->sendMessageToClient("FocusPosition:" + QString::number(CurrentPosition) + ":" + QString::number(CurrentPosition));
            return;
        }
        if (CurrentPosition == lastPosition){
            indi_Client->abortFocuserMove(dpFocuser);
            focusMoveToMaxorMinTimer->stop();
            emit wsThread->sendMessageToClient("focusMoveFailed:check if the focuser is stuck or at the physical limit");
            return;
        }

        lastPosition = CurrentPosition; });
    focusMoveToMaxorMinTimer->start(1000);
    Logger::Log("focusMoveToMin | Started moving to minimum position: " + std::to_string(min), LogLevel::INFO, DeviceType::FOCUSER);
}

void MainWindow::focusMoveToMax()
{
    if (dpFocuser == nullptr)
    {
        Logger::Log("focusMoveToMax | dpFocuser is nullptr", LogLevel::ERROR, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient("focusMoveFailed:focuser is not connected");
        return;
    }
    indi_Client->abortFocuserMove(dpFocuser);
    int timeout = 0;
    while (timeout <= 2)
    {
        CurrentPosition = FocuserControl_getPosition();
        emit wsThread->sendMessageToClient("FocusPosition:" + QString::number(CurrentPosition) + ":" + QString::number(CurrentPosition));
        sleep(1);
        timeout++;
    }
    CurrentPosition = FocuserControl_getPosition();
    emit wsThread->sendMessageToClient("FocusPosition:" + QString::number(CurrentPosition) + ":" + QString::number(CurrentPosition));
    // 更新最小范围为当前位置
    focuserMinPosition = CurrentPosition;
    int min, max, step, value;
    indi_Client->getFocuserRange(dpFocuser, min, max, step, value); // 获取焦点器范围
    // indi_Client->moveFocuserToAbsolutePosition(dpFocuser, max); // 移动到最大位置
    indi_Client->setFocuserMoveDiretion(dpFocuser, false);
    int steps = std::min(max + CurrentPosition, 60000);
    indi_Client->moveFocuserSteps(dpFocuser, steps);
    TargetPosition = CurrentPosition + steps;
    if (focusMoveToMaxorMinTimer != nullptr)
    {
        focusMoveToMaxorMinTimer->stop();
        focusMoveToMaxorMinTimer->deleteLater();
        focusMoveToMaxorMinTimer = nullptr;
    }
    focusMoveToMaxorMinTimer = new QTimer(this);
    CurrentPosition = FocuserControl_getPosition();
    lastPosition = CurrentPosition;
    connect(focusMoveToMaxorMinTimer, &QTimer::timeout, this, [this, max]()
            {
        CurrentPosition = FocuserControl_getPosition();
        emit wsThread->sendMessageToClient("FocusPosition:" + QString::number(CurrentPosition) + ":" + QString::number(CurrentPosition));
        if (CurrentPosition == TargetPosition){
            int min, max, step, value;
            indi_Client->getFocuserRange(dpFocuser, min, max, step, value); // 获取焦点器范围
            int steps = std::min(max-CurrentPosition,60000);
            indi_Client->moveFocuserSteps(dpFocuser, steps);
            TargetPosition = CurrentPosition+steps;
            emit wsThread->sendMessageToClient("FocusPosition:" + QString::number(CurrentPosition) + ":" + QString::number(CurrentPosition));
            return;
        }
        if (CurrentPosition == lastPosition){
            indi_Client->abortFocuserMove(dpFocuser);
            focusMoveToMaxorMinTimer->stop();
            emit wsThread->sendMessageToClient("focusMoveFailed:check if the focuser is stuck or at the physical limit");
            return;
        }
        lastPosition = CurrentPosition; });
    focusMoveToMaxorMinTimer->start(1000);
    Logger::Log("focusMoveToMax | Started moving to maximum position: " + std::to_string(max), LogLevel::INFO, DeviceType::FOCUSER);
}

void MainWindow::focusSetTravelRange()
{
    if (dpFocuser == nullptr)
    {
        Logger::Log("focusSetTravelRange | dpFocuser is nullptr", LogLevel::ERROR, DeviceType::FOCUSER);
        emit wsThread->sendMessageToClient("focusMoveFailed:focuser is not connected");
        return;
    }
    if (focusMoveToMaxorMinTimer != nullptr)
    {
        focusMoveToMaxorMinTimer->stop();
        focusMoveToMaxorMinTimer->deleteLater();
        focusMoveToMaxorMinTimer = nullptr;
    }
    indi_Client->abortFocuserMove(dpFocuser);
    int timeout = 0;
    while (timeout <= 2)
    {
        CurrentPosition = FocuserControl_getPosition();
        emit wsThread->sendMessageToClient("FocusPosition:" + QString::number(CurrentPosition) + ":" + QString::number(CurrentPosition));
        sleep(1);
        timeout++;
    }
    CurrentPosition = FocuserControl_getPosition();
    focuserMaxPosition = CurrentPosition;
    emit wsThread->sendMessageToClient("focusSetTravelRangeSuccess");
    emit wsThread->sendMessageToClient("FocuserLimit:" + QString::number(focuserMinPosition) + ":" + QString::number(focuserMaxPosition));
    Tools::saveParameter("Focuser", "focuserMaxPosition", QString::number(focuserMaxPosition));
    Tools::saveParameter("Focuser", "focuserMinPosition", QString::number(focuserMinPosition));
}

void MainWindow::getCheckBoxSpace()
{
    // 计算应用所在分区的可用空间，避免与实际存储分区不一致
    QString path = QCoreApplication::applicationDirPath();
#ifdef Q_OS_WIN
    path = QDir::rootPath();
#endif
    QFileInfo fi(path);
    quint64 freeBytes = 0;
    if (fi.exists())
    {
        QStorageInfo storage(path);
        freeBytes = static_cast<quint64>(storage.bytesAvailable());
    }
    // 发送到前端：Box_Space:<bytes>
    if (wsThread)
    {
        emit wsThread->sendMessageToClient("Box_Space:" + QString::number(freeBytes));
    }
}

void MainWindow::clearLogs()
{
    // 按 Logger::Initialize 约定：运行目录下 logs/MAIN.log 等
    const QString logDirPath = QDir::currentPath() + "/logs";
    QDir logDir(logDirPath);
    if (logDir.exists())
    {
        QFileInfoList entries = logDir.entryInfoList(QDir::Files | QDir::NoDotAndDotDot);
        for (const QFileInfo &fi : entries)
        {
            // 清空内容而不删除文件
            QFile f(fi.absoluteFilePath());
            if (f.open(QIODevice::WriteOnly | QIODevice::Truncate))
            {
                f.close();
            }
        }
    }
    if (wsThread) emit wsThread->sendMessageToClient("ClearLogs:Success");
}

void MainWindow::clearBoxCache(bool clearCache, bool clearUpdatePack, bool clearBackup)
{
    auto clearDirContents = [](const QString &dirPath)
    {
        if (dirPath.isEmpty()) return;
        QDir dir(dirPath);
        if (!dir.exists()) return;
        // 删除目录下的所有条目（文件/链接/目录），保留顶层目录本身
        QFileInfoList entries = dir.entryInfoList(QDir::NoDotAndDotDot | QDir::AllEntries | QDir::Hidden);
        for (const QFileInfo &fi : entries)
        {
            if (fi.isSymLink() || fi.isFile())
            {
                QFile::remove(fi.absoluteFilePath());
            }
            else if (fi.isDir())
            {
                QDir sub(fi.absoluteFilePath());
                sub.removeRecursively();
            }
        }
    };

    // 1. 清理系统/应用缓存与回收站
    if (clearCache)
    {
        QStringList caches;
        // 用户主目录缓存根（比单纯的 CacheLocation 更全面）
        caches << (QDir::homePath() + "/.cache");
        caches << QStandardPaths::writableLocation(QStandardPaths::GenericCacheLocation)
               << QStandardPaths::writableLocation(QStandardPaths::CacheLocation)
               << QStandardPaths::writableLocation(QStandardPaths::TempLocation);
        // XDG 垃圾箱（当前用户）常规路径与 XDG_DATA_HOME 路径
        const QString trashBase = QDir::homePath() + "/.local/share/Trash";
        caches << (trashBase + "/files") << (trashBase + "/info");
        const QString xdgDataHome = qEnvironmentVariableIsSet("XDG_DATA_HOME")
                                    ? QString::fromLocal8Bit(qgetenv("XDG_DATA_HOME"))
                                    : (QDir::homePath() + "/.local/share");
        caches << (xdgDataHome + "/Trash/files") << (xdgDataHome + "/Trash/info");
        // 常见桌面环境的垃圾箱路径（可能存在）
        caches << (QDir::homePath() + "/.Trash")
               << (QDir::homePath() + "/.Trash-1000/files")
               << (QDir::homePath() + "/.Trash-1000/info");

        for (const QString &p : caches) clearDirContents(p);

        // 尝试调用 gio 清空垃圾箱（若环境支持）
        QProcess::execute("gio", QStringList() << "trash" << "--empty");

        // 清空可移动介质等可能挂载点的垃圾箱（.Trash-UID 或 .Trash）
        QString uidStr = QString::number(getuid());
        QStringList mountRoots;
        mountRoots << "/mnt" << "/media" << "/run/media";
        for (const QString &root : mountRoots)
        {
            QDir rootDir(root);
            if (!rootDir.exists()) continue;
            QFileInfoList subs = rootDir.entryInfoList(QDir::NoDotAndDotDot | QDir::Dirs);
            for (const QFileInfo &fi : subs)
            {
                const QString base = fi.absoluteFilePath();
                clearDirContents(base + "/.Trash-" + uidStr + "/files");
                clearDirContents(base + "/.Trash-" + uidStr + "/info");
                clearDirContents(base + "/.Trash/files");
                clearDirContents(base + "/.Trash/info");
            }
        }
    }

    // 2. 可选：清理更新包目录
    if (clearUpdatePack)
    {
        clearDirContents("/var/www/update_pack");
    }

    // 3. 可选：清理备份目录
    if (clearBackup)
    {
        clearDirContents("~/workspace/QUARCS/backup");
    }

    if (wsThread) emit wsThread->sendMessageToClient("ClearBoxCache:Success");
}

void MainWindow::getFocuserParameters()
{
    QMap<QString, QString> parameters = Tools::readParameters("Focuser");
    if (parameters.contains("focuserMaxPosition") && parameters.contains("focuserMinPosition"))
    {
        focuserMaxPosition = parameters["focuserMaxPosition"].toInt();
        focuserMinPosition = parameters["focuserMinPosition"].toInt();
    }
    else
    {
        focuserMaxPosition = -1;
        focuserMinPosition = -1;
    }
    emit wsThread->sendMessageToClient("FocuserLimit:" + QString::number(focuserMinPosition) + ":" + QString::number(focuserMaxPosition));
    Logger::Log("Focuser Max Position: " + std::to_string(focuserMaxPosition) + ", Min Position: " + std::to_string(focuserMinPosition), LogLevel::INFO, DeviceType::MAIN);
    Logger::Log("Focuser Current Position: " + std::to_string(CurrentPosition), LogLevel::INFO, DeviceType::MAIN);

    // 空程
    int emptyStep = parameters.contains("Backlash") ? parameters["Backlash"].toInt() : 0;
    autofocusBacklashCompensation = emptyStep;
    emit wsThread->sendMessageToClient("Backlash:" + QString::number(emptyStep));
    
    // 粗调分段数（总行程 / 分段数）
    int coarseDivisions = parameters.contains("coarseStepDivisions") ? parameters["coarseStepDivisions"].toInt() : 10;
    if (coarseDivisions <= 0)
    {
        coarseDivisions = 10;
    }
    autoFocusCoarseDivisions = coarseDivisions;
    emit wsThread->sendMessageToClient("Coarse Step Divisions:" + QString::number(autoFocusCoarseDivisions));

    // 将当前 Focuser 串口列表与已保存串口下发给前端（若无保存则 savedPort 为空）
    sendSerialPortOptions("Focuser");

}

void MainWindow::getFocuserState()
{

    QString state = isAutoFocus ? "true" : "false";
    emit wsThread->sendMessageToClient("updateAutoFocuserState:" + state); // 状态更新

    // 获取当前步骤
    if (isAutoFocus && autoFocus != nullptr)
    {
        emit wsThread->sendMessageToClient("AutoFocusStarted:自动对焦已开始");
        autoFocus->getAutoFocusStep();
    }

    // 获取当前点和线数据
    if (isAutoFocus && autoFocus != nullptr)
    {
        autoFocus->getAutoFocusData();
    }
}
