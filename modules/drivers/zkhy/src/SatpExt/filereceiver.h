#ifndef FILERECEIVER_H
#define FILERECEIVER_H

#include <QString>
#include <QVector>
#include "satpext_global.h"
#include "blockhandler.h"

class QFile;

namespace SATP {

class Protocol;
class FileReceiverHandler;

class SATPEXTSHARED_EXPORT FileReceiver : public BlockHandler
{
public:
    explicit FileReceiver(Protocol *protocol);
    virtual ~FileReceiver();
    FileReceiver(const FileReceiver&&){}
    //override.
    bool handleReceiveBlock(uint32_t dataType, const char *block, int size);

    //FileReceiver
    void registerReceiverHandler(FileReceiverHandler *receiverHandler);
    double getReceiveProgress();
    void setRecvFileDir(const QString &dir);

protected:
    void handleFileHeader(const char *block);
    void handleFileTail(const char *block);
    void handleFileData(const char *block, int size);
    void sendFileResp(bool finished = false);
    void raiseReceiverHandlers(const QString &filePath);

private:
    QVector<FileReceiverHandler *> mReceiverHandlers;
    Protocol     *mProtocol;

    int           mFileSize;
    QFile        *mFile;
    int           mReceived;
    int           mPacketCount;
    QString       mRecvFileDir;
};

}
#endif // FILERECEIVER_H
