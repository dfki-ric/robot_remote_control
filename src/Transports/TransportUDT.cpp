#include "TransportUDT.hpp"
#include <iostream>
#include <cstring> //memset
#include <unistd.h>

using namespace std;
using namespace robot_remote_control;


TransportUDT::TransportUDT(const ConnectionType &type, const int &port, const std::string &addr, size_t recvBufferSize):connectiontype(type),port(port),addr(addr),recvBufferSize(recvBufferSize){

    blockingRecv = true;
    blockingSend = true;
    serv = 0;
    recvBuffer.resize(recvBufferSize);

    UDT::startup();

    if (type == SERVER){

        serv = UDT::socket(AF_INET, SOCK_DGRAM, 0);

        bool block = true;
        UDT::setsockopt(serv, 0 /*ignored*/, UDT_RCVSYN,&block,sizeof(bool));
        bool reuse = true;
        UDT::setsockopt(serv, 0 /*ignored*/, UDT_REUSEADDR,&reuse, sizeof(bool));


        sockaddr_in my_addr;
        my_addr.sin_family = AF_INET;
        my_addr.sin_port = htons(port);
        my_addr.sin_addr.s_addr = INADDR_ANY;
        memset(&(my_addr.sin_zero), '\0', 8);

        if (UDT::ERROR == UDT::bind(serv, (sockaddr*)&my_addr, sizeof(my_addr)))
        {
            cout << "bind: " << UDT::getlasterror().getErrorMessage();
        }

        UDT::listen(serv, 10);

        socket.set(0);

        //start connect thread (locks socket)
        acceptthread = std::thread(&TransportUDT::accept,this);

        //sleep(1);

    }else{
        socket.set(UDT::socket(AF_INET, SOCK_DGRAM, 0));

        // bool block = true;
        // socket.lock();
        // UDT::setsockopt(socket.get_ref(), 0 /*ignored*/, UDT_RCVSYN,&block,sizeof(bool));
        // socket.unlock();

        acceptthread = std::thread(&TransportUDT::connect,this);
        //sleep(1);
    }
        
}

TransportUDT::~TransportUDT(){

    acceptthread.join();

    if (socket.get()){
        UDT::close(socket.get());
    }
    if (serv){
        UDT::close(serv);
    }
}


void TransportUDT::accept(){
    auto lockedSocket = socket.getLockedAccess();
//      if (!socket.get_ref()){
        cout << __PRETTY_FUNCTION__ << port << endl;

        int namelen;
        sockaddr_in their_addr;

        lockedSocket.get() = UDT::accept(serv, (sockaddr*)&their_addr, &namelen);

        UDT::setsockopt(lockedSocket.get(), 0 /*ignored*/, UDT_RCVSYN,&blockingRecv,sizeof(bool));
        UDT::setsockopt(lockedSocket.get(), 0 /*ignored*/, UDT_SNDSYN,&blockingSend,sizeof(bool));

        int recvtimeout = 500; //ms
        UDT::setsockopt(lockedSocket.get(), 0 /*ignored*/, UDT_SNDTIMEO,&recvtimeout, sizeof(int));
        UDT::setsockopt(lockedSocket.get(), 0 /*ignored*/, UDT_RCVTIMEO,&recvtimeout, sizeof(int));

        cout << "new connection: " << inet_ntoa(their_addr.sin_addr) << ":" << ntohs(their_addr.sin_port) << endl;
//    }
}

void TransportUDT::connect(){
    auto lockedSocket = socket.getLockedAccess();
    sockaddr_in serv_addr;
    serv_addr.sin_family = AF_INET;
    serv_addr.sin_port = htons(port);
    inet_pton(AF_INET, addr.c_str(), &serv_addr.sin_addr);
    memset(&(serv_addr.sin_zero), '\0', 8);

    cout << __PRETTY_FUNCTION__ << port << endl;
    // connect to the server, implict bind
    if (UDT::ERROR == UDT::connect(lockedSocket.get(), (sockaddr*)&serv_addr, sizeof(serv_addr)))
    {
        cout << "connect: " << UDT::getlasterror().getErrorMessage();
    }
//    int recvtimeout = 500; //ms
//    UDT::setsockopt(socket.get_ref(), 0 /*ignored*/, UDT_RCVTIMEO,&recvtimeout, sizeof(int));
//    UDT::setsockopt(socket.get_ref(), 0 /*ignored*/, UDT_SNDTIMEO,&recvtimeout, sizeof(int));
    UDT::setsockopt(lockedSocket.get(), 0 /*ignored*/, UDT_RCVSYN,&blockingRecv,sizeof(bool));
    UDT::setsockopt(lockedSocket.get(), 0 /*ignored*/, UDT_SNDSYN,&blockingSend,sizeof(bool));
} 


int TransportUDT::send(const std::string& buf, Flags flags){
    auto lockedSocket = socket.getLockedAccess();

    if ((flags & NOBLOCK) && blockingSend ){
        //if should not block but enabled:
        blockingSend = false;
        UDT::setsockopt(lockedSocket.get(), 0 /*ignored*/, UDT_SNDSYN,&blockingSend,sizeof(bool));
    }else if ((!(flags & NOBLOCK)) && !blockingSend ){
        //if should block but disabled:
        blockingSend = true;
        UDT::setsockopt(lockedSocket.get(), 0 /*ignored*/, UDT_SNDSYN,&blockingSend,sizeof(bool));
    }

    int sent = UDT::sendmsg(lockedSocket.get(), buf.data(), buf.size() , -1, true); //ttl -1, inorder true
    if (UDT::ERROR == sent)
    {
        cout << "send error: " << UDT::getlasterror().getErrorMessage();
        return 0;
    }
    //cout << "send done " << addr << ":" << port << " bytes:" << sent << " " << connectiontype << std::endl;
    return sent;
}

int TransportUDT::receive(std::string* buf, Flags flags){
    auto lockedSocket = socket.getLockedAccess();
    
    if ((flags & NOBLOCK) && blockingRecv ){
        //if should not block but enabled:
        blockingRecv = false;
        UDT::setsockopt(lockedSocket.get(), 0 /*ignored*/, UDT_SNDSYN,&blockingRecv,sizeof(bool));
    }else if ((!(flags & NOBLOCK)) && !blockingRecv ){
        //if should block but disabled:
        blockingRecv = true;
        UDT::setsockopt(lockedSocket.get(), 0 /*ignored*/, UDT_SNDSYN,&blockingRecv,sizeof(bool));
    }

    int received = UDT::recvmsg(lockedSocket.get(), (char*)recvBuffer.data(), recvBuffer.size());
    if (UDT::ERROR == received)
    {

        switch(UDT::getlasterror().getErrorCode())
        {
            case 6002: // no data is available to be received on a non-blocking socket
            case 6003: // receive timeout
                break;
            case 2001: // connection broken before send is completed
            case 2002: // not connected
            case 5004: // invalid UDT socket
            case 5009: // wrong mode
            case 6004: // an overlapped recv is in progress
            default:
                //throw std::runtime_error("TransportUDT receive: " + std::string(UDT::getlasterror().getErrorMessage()));        
                cout << UDT::getlasterror().getErrorMessage() << " code :" << UDT::getlasterror().getErrorCode() << endl;
        }
            return 0;
    }
    //cout << "receive done: "  << port << " bytes:" << received << " " << connectiontype  << std::endl;

    //don't use string assign here, no need to inti the whole buffer size on target string
    buf->resize(received);//should already be the case
    memcpy((void*)buf->data(),recvBuffer.data(),received);
    
    return received;
}


