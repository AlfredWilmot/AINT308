#ifndef OWLCOMMS_H
#define OWLCOMMS_H

#endif // OWLCOMMS_H

// TCP socket communications
/*
 * Phil CULverhouse Oct 2016 (c) Plymouth UNiversity
 *
 * this version uses an IP sockets python script on the Owl (server) to control the
 * servos using a 24 byte long packet.
 * [Ry Rx Ly Lx Neck] each in the range 1000 to 2000 (see below for crude calibration)
 * without video streaming this can run flat out at 10 owl command per half second
 * this includes sending the position string, the server range checks the PWM
 * to ensure no over driving of thme servos, and
 * receiving a 2 byte 'OK' packet from the server and closing the connection
 * with video streaming operating as well, this drops to 1.5 seconds per command
 *
 * http://blog.pusheax.com/2013/07/windows-api-winsock-create-your-first.html for winsock2 example
 * as linux and win32 have different IP socket calling structures
 */
#ifdef _WIN32
# include <winsock2.h>
# include <windows.h>
#else
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include <arpa/inet.h>
#define MSG_NOSIGNAL 0x80 // PFC March 2017 -- see https://lists.apple.com/archives/macnetworkprog/2002/Dec/msg00091.html
#ifndef SOCKET
typedef int SOCKET; // PFC mar 2017 WIN32 has it defined, linux not
#endif

#endif

#include <iostream> // for standard I/O
#include <string>   // for strings

#define SERVER_PORT htons(PORT) // set to be same as the Owl Pi server

using namespace std;

static int clientSock;

#ifdef _WIN32
SOCKET OwlCommsInit (int PORT, string PiADDR ){
    WSAData version;        //We need to check the version.
    WORD mkword=MAKEWORD(2,2);
    int what=WSAStartup(mkword,&version);
    if(what!=0){
        std::cout<<"This version is not supported! - \n"<<WSAGetLastError()<<std::endl;
    }
    else{
        std::cout<<"Good - Everything fine!\n"<<std::endl;
    }

    SOCKET u_sock=socket(AF_INET,SOCK_STREAM,IPPROTO_TCP);
    if(u_sock==INVALID_SOCKET)
        std::cout<<"Creating socket fail\n";

    else
        std::cout<<"It was okay to create the socket\n";
    //Socket address information
    sockaddr_in addr;
    addr.sin_family=AF_INET;
    addr.sin_addr.s_addr=inet_addr(PiADDR.c_str());
    addr.sin_port=htons(PORT); // HAS to match that defined in Pi server
    /*==========Addressing finished==========*/

    //Now we connect
    int conn=connect(u_sock,(SOCKADDR*)&addr,sizeof(addr));
    if(conn==SOCKET_ERROR){
        std::cout<<"Error - when connecting "<<WSAGetLastError()<<std::endl;
        closesocket(u_sock);
        WSACleanup();
    }
    return (u_sock); //PFC Mar 2017 moved from below
}
#else
int OwlCommsInit (int PORT, string PiADDR ){
    clientSock=socket(AF_INET,SOCK_STREAM, 0);
    sockaddr_in serverAddress;
    serverAddress.sin_family=AF_INET;
    serverAddress.sin_port=SERVER_PORT;
    serverAddress.sin_addr.s_addr=inet_addr(PiADDR.c_str());

    connect(clientSock, (struct sockaddr*)&serverAddress, sizeof(struct sockaddr));
    return(clientSock); //PFC added Mar 2017
}
#endif

#ifdef _WIN32
string OwlSendPacket (SOCKET u_sock, string CMD){
    char receivedCHARS[3] = {0,0,0}; // send 'ok' back

    int smsg=send(u_sock,CMD.c_str(),strlen(CMD.c_str()),0);
    if(smsg==SOCKET_ERROR){
        std::cout<<"Error: "<<WSAGetLastError()<<std::endl;
        WSACleanup();
    }
    int N=recv(u_sock,receivedCHARS,2,0);
    if(N==SOCKET_ERROR){
        std::cout<<"Error in Receiving: "<<WSAGetLastError()<<std::endl;
    }
    std::cout<<"OK:" << N <<std::endl;
    return ("OK:" );
}
#else
string OwlSendPacket (int clientSock, string CMD){
    char receivedCHARS[3] = {0,0,0}; // send 'ok' back
    send(clientSock, CMD.c_str(), 24, MSG_NOSIGNAL); // make a broken pipe error rather than crash added MSG_NOSIGNAL PFC March 2017
    int N = recv(clientSock, &receivedCHARS, 2, 0);
    //cout << "Server received:  " << receivedCHARS << " N:" << N << endl;
    return ("OK:" );
}
#endif
