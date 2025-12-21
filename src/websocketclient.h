#ifndef WEBSOCKETCLIENT_H
#define WEBSOCKETCLIENT_H

#include <QObject>
#include <QWebSocket>
#include <QJsonDocument>
#include <QJsonObject>
#include <QUrl>
#include <QTimer>
#include <QElapsedTimer>
#include <QQueue>
#include <QAbstractSocket>
#include <QSslError>
#include <QSslConfiguration>

// Note: QJsonDocument/QJsonObject inclusions are not necessary here because they are internal types
// in the .cpp.

class WebSocketClient : public QObject
{
    Q_OBJECT
public:
    explicit WebSocketClient(const QUrl &httpUrl, const QUrl &httpsUrl, QObject *parent = nullptr);
    void messageSend(QString message);
    void sendProcessCommandReturn(QString message);
    void sendAcknowledgment(QString messageID);
    void stop();

signals:
    void messageReceived(const QString &message);
    void connectionStatusChanged(bool isConnected, const QString &statusMessage);
    void connectionError(const QString &errorMessage);

private slots:
    // HTTPS Slots
    void onHttpsConnected();
    void onHttpsDisconnected();
    void onSslErrors(const QList<QSslError> &errors);

    // HTTP Slots
    void onHttpConnected();
    void onHttpDisconnected();

    // Common Slots
    void onError(QAbstractSocket::SocketError error);
    void reconnect();
    void onTextMessageReceived(QString message);
    void onHeartbeatTimeout();
    void onPongReceived(quint64 elapsedTime, const QByteArray &payload);
    // [DELETED] void onNetworkStateChanged(bool isOnline); - Replaced by the logic of reconnection

private:
    void flushPending();
    QString getErrorString(QAbstractSocket::SocketError error);

private:
    QWebSocket httpsWebSocket;
    QWebSocket httpWebSocket;
    QUrl httpUrl;
    QUrl httpsUrl;

    // State
    bool isHttpsConnected;
    bool isHttpConnected;
    // [SUPPRIMÉ] bool isNetworkConnected;
    // [SUPPRIMÉ] QNetworkConfigurationManager networkManager;

    // Reconnection Logic
    QTimer reconnectTimer;
    int reconnectAttempts = 0;
    int currentReconnectIntervalMs = 1000;
    const int maxReconnectIntervalMs = 60000;

    // Heartbeat/Pong Logic
    QTimer heartbeatTimer;
    QElapsedTimer lastPongTimer;
    const int heartbeatIntervalMs = 5000;
    const int pongTimeoutMultiplier = 3; // 15s timeout
    int missedPongs = 0;

    // Pending Messages
    QQueue<QByteArray> pendingMessages;
    const int maxPendingMessages = 1000;
};

#endif // WEBSOCKETCLIENT_H
