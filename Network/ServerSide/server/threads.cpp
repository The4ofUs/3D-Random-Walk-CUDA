#include <stdio.h>
#include "threads.h"
#include <QTime>
#include <iostream>
#include <QDir>
#include <QtWidgets>
#include "serverside.h"
#include"initiateServer.h"
#include<QVector>
#include "Photon.h"
#include <QDebug>
int Detected;
int Terminated;
QVector<Photon> receivedResults;
int counter=0;
int photonsPerPatch;
float detectorRadius = 10;
float tissueRadius = 10;
float tissueAbsCoeff = 1;
float tissueScatCoeff = 100 ;
Point *detectorPosition = new Point(0,0,50);
Point *tissueFirstCenter = new Point(0,0,50);
Point *tissueSecondCenter = new Point(0,0,-50);


threads::threads(int ID, QObject *parent):
    QThread(parent)
{
    this->socketDescriptor = ID;
}


void  threads::run()
{
    qDebug() << "Starting new thread";
    socket = new QTcpSocket();
    if (!socket->setSocketDescriptor(this->socketDescriptor))
    {
        emit error(socket->error());
        return;
    }

    //qDebug()<<"Read on  readyRead";
    connect(socket, SIGNAL(readyRead()),this, SLOT(read()),Qt::DirectConnection);
    connect(socket, SIGNAL(disconnected()),this, SLOT(disconnected()),Qt::DirectConnection);
    qDebug() <<"client with socket ="<< socketDescriptor << "is Connected!";
    dataSize=0;
    exec();
}


void threads::read(){
    QByteArray queryTypeByteArr = socket->readAll();
    queryType = queryTypeByteArr.toStdString();
    if(queryType.compare("requestParameters")==0){
        qDebug()<<"client with socket ="<<socketDescriptor<< "is requesting parameters";
        sendParameters();
    }
    else if(queryType.compare("requestBatch")==0){
        qDebug()<<"client with socket ="<<socketDescriptor<<"is requesting new batch";
        sendNewBatch();
    }
    else if(queryType.compare("prepareForReceiving")==0){
        qDebug() <<"client with socket ="<< socketDescriptor<<"is sending results";
        disconnect(socket, SIGNAL(readyRead()),this, SLOT(read()));
        connect(socket,SIGNAL(readyRead()),this,SLOT(readPhotonsVector()),Qt::DirectConnection);
        socket->write("readyToReceive");
    }
}

void threads::sendNewBatch(){
    QByteArray newBatchByteArray;
    QDataStream newBatch(&newBatchByteArray,QIODevice::WriteOnly);
    newBatch.setVersion(QDataStream::Qt_4_8);
    if(batchRemainingPhotons==0){
        photonsPerPatch=0;
        //abort connection, close server and average results
        qDebug()<<"Server has no more batches";
    }
    newBatch<< photonsPerPatch;
    socket->write(newBatchByteArray);
    newBatchSignal();
}


void threads::getBatchremainingPhotons(int batches){
    batchRemainingPhotons = batches;
}

void threads::getNumberOfPhotons(int numPhotons)
{
    photonsPerPatch = numPhotons;
}

void threads::sendParameters(){
    QVector<float> userParameters;
    QByteArray parametersByteArray;
    QDataStream paramtersTobeSend(&parametersByteArray,QIODevice::WriteOnly);
    paramtersTobeSend.setVersion(QDataStream::Qt_4_8);
    userParameters.append((float)photonsPerPatch);
    userParameters.append(detectorRadius);
    userParameters.append((float)detectorPosition->x());
    userParameters.append((float)detectorPosition->y());
    userParameters.append((float)detectorPosition->z());
    userParameters.append(tissueRadius);
    userParameters.append(tissueAbsCoeff);
    userParameters.append(tissueScatCoeff);
    userParameters.append((float)tissueFirstCenter->x());
    userParameters.append((float)tissueFirstCenter->y());
    userParameters.append((float)tissueFirstCenter->z());
    userParameters.append((float)tissueSecondCenter->x());
    userParameters.append((float)tissueSecondCenter->y());
    userParameters.append((float)tissueSecondCenter->z());
    paramtersTobeSend << userParameters;
    socket->write(parametersByteArray);
    qDebug()<<"Parameters are sent";
    newBatchSignal();

}

QByteArray array ;
void threads::readPhotonsVector(){
    qDebug()<<"read photons on readyRead";
    socket->waitForReadyRead();
    qDebug()<<"Bytes Available"<< socket->bytesAvailable();
    if( dataSize == 0 )
    {
        QDataStream stream(socket);
        stream.setVersion(QDataStream::Qt_4_8);
        if( socket->bytesAvailable() < sizeof(quint32) )
            return;
        stream >> dataSize;
    }
    if( dataSize > socket->bytesAvailable() )
        return;
    do{
        array += socket->readAll();
    }
    while(array.size()<dataSize);
    QVector<float> X;
    QVector<float> Y;
    QVector<float> Z;
    QVector<float> W;
    QVector<float> ST;
    // qDebug()<<"Recived Array"<<array.size();
    QDataStream streamm(&array, QIODevice::ReadOnly);
    streamm.setVersion(QDataStream::Qt_4_8);

    if (array.size()==dataSize*8+20){
        qDebug()<<"Recived Array total size"<<array.size();
        streamm >> X>>Y>>Z>>W>>ST;
        qDebug()<<"Random Point";
        qDebug()<<X[5];
        //qDebug()<<X.size() <<Y.size()<< Z.size()<<W.size()<<ST.size();
        qDebug()<<"Results are recived";
        //initiateServer->appendToVectors(X, Y, Z, W, ST);
        array.clear();
    }
    reconditionResultsToPhotons(X,Y,Z,W,ST);
    qDebug()<<"results after conditioning"<<receivedResults.size();
    appendNewReceivedResultsSignal();
}

QVector<Photon> threads::returnRecievedPhotons()
{
    qDebug()<<"at return"<<receivedResults.size();

    return receivedResults;
}


void threads::reconditionResultsToPhotons(QVector<float> x,QVector<float> y, QVector<float> z, QVector<float> w, QVector<float> s){
    receivedResults.clear();
    for(int i=0; i<x.size();i++){
        Photon ph;
        Point P;
        P.setCoordinates(x[i],y[i],z[i]);
        ph.setPosition(P);
        ph.setWeight(w[i]);
        ph.setState(s[i]);
        receivedResults.push_back(ph);
    }
    qDebug()<<"Recevied results reconditioned to array of photons size is "<<receivedResults.size();
}

void threads::disconnected()
{

    qDebug() <<"client with socket ="<< socketDescriptor << "Disconnected";
    socket->deleteLater();
    exit(0);
}


int threads::DetectedCounter(){
    return Detected;
}


int threads::terminatedCounter()
{
    return Terminated;
}
