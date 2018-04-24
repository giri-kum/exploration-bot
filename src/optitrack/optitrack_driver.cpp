#include <optitrack/optitrack.hpp>
#include <optitrack/optitrack_channels.h>
#include <optitrack/common/getopt.h>
#include <optitrack/common/timestamp.h>
#include <lcmtypes/balancebot_msg_t.hpp>
#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/balancebot_gate_t.hpp>
#include <lcm/lcm-cpp.hpp>
#include <netinet/in.h>
#include <sys/socket.h>
#include <iostream>
#include <cmath>


int main(int argc, char** argv)
{
    const char* kInterfaceArg = "interface";
    const char* kBBRigidBodyArg = "balancebot";
    const char* kNumGatesArg = "number of gates";
    
    getopt_t* gopt = getopt_create();
    getopt_add_bool(gopt, 'h', "help", 0, "Display this help message.\n");
    getopt_add_string(gopt, 'i', kInterfaceArg, "192.168.3.0", "Local network interface used when connecting to the Optitrack network. (Ex. inet as shown by ifconfig on the current machine.)");
    getopt_add_int(gopt, 'r', kBBRigidBodyArg, "5", "Id of Balancebot rigid body to publish pose for.");
    getopt_add_int(gopt, 'n', kNumGatesArg, "4", "Number of Gates (IDs 1-N)");

    
    
    if (!getopt_parse(gopt, argc, argv, 1) || getopt_get_bool(gopt, "help")) {
        printf("Usage: %s [options]", argv[0]);
        getopt_do_usage(gopt);
        return 1;
    }
    
    std::string interface = getopt_get_string(gopt, kInterfaceArg);
    int BBrigidBodyId = getopt_get_int(gopt, kBBRigidBodyArg);
    int NumGates = getopt_get_int(gopt, kNumGatesArg);



    
    // If there's no interface specified, then we'll need to guess
    if (interface.length() == 0)
    {
        interface = guess_optitrack_network_interface();
    }
    // If there's still no interface, we have a problem
    if (interface.length() == 0)
    {
        printf("[optitrack_driver] error could not determine network interface for receiving multicast packets.\n");
        return -1;
    }
    
    SOCKET dataSocket = create_optitrack_data_socket(interface, PORT_DATA);
    
    if (dataSocket == -1) {
        printf("[optitrack_driver] error failed to create socket for interface %s:%d\n", interface.c_str(), PORT_DATA);
        return -1;
    } else {
        printf("[optitrack_driver] successfully created socket for interface %s:%d\n", interface.c_str(), PORT_DATA);
    }
    
    // Code from DataListenThread function in PacketClient.cpp
    char packet[20000];
    socklen_t addrLen = sizeof(sockaddr);
    sockaddr_in incomingAddress;
    std::vector<optitrack_message_t> incomingMessages;
    
    lcm::LCM lcmInstance;
    // MAIN THREAD LOOP

    printf("\nOptitrack Driver Running...\n");
    while (1) {
        balancebot_msg_t BBmsg;
        BBmsg.num_gates = NumGates;
        // Block until we receive a datagram from the network
        recvfrom(dataSocket, packet, sizeof(packet), 0, (sockaddr*)&incomingAddress, &addrLen);
        incomingMessages = parse_optitrack_packet_into_messages(packet, sizeof(packet));
        for(auto& msg : incomingMessages) {
            if (msg.id <= NumGates)
            {
                int pair = 0;
                float m[3][2];
                balancebot_gate_t gate;
                int i;
                for(i=0;i<3;i++){
                    m[i][0] = msg.markers[i].x;
                    m[i][1] = msg.markers[i].z;
                    //printf("Marker %d: (%f, %f)\n",i,m[i][0],m[i][1]);
                }
                for(i=0; i<2;i++){
                    float xd = m[i][0]-m[i+1][0];
                    float yd = m[i][1]-m[i+1][1];
                    float d = sqrt(xd*xd+yd*yd);
                    //printf("Dist (%d,%d) = %f\n",i,i+1,d);
                    if( d < 0.2){
                        pair = i+1;
                        break;
                    }
                }

                if(pair == 1){ // left = (m0, m1)
                    gate.left_post[0] = (m[0][0]+m[1][0])/2;
                    gate.left_post[1] = (m[0][1]+m[1][1])/2;
                    gate.right_post[0] = m[2][0];
                    gate.right_post[1] = m[2][1];
                }
                else if(pair == 2){ // left = (m1, m2)
                    gate.left_post[0] = (m[1][0]+m[2][0])/2;
                    gate.left_post[1] = (m[1][1]+m[2][1])/2;
                    gate.right_post[0] = m[0][0];
                    gate.right_post[1] = m[0][1];
                }
                
                else{ // left = (m0, m2)
                    gate.left_post[0] = (m[0][0]+m[2][0])/2;
                    gate.left_post[1] = (m[0][1]+m[2][1])/2;
                    gate.right_post[0] = m[1][0];
                    gate.right_post[1] = m[1][1];
                }
                //printf("ID:%d, left=%d\n",msg.id,pair);
                BBmsg.gates.push_back(gate); 
            }

            else if(msg.id == BBrigidBodyId) {
                pose_xyt_t Pose;
                Pose.utime = utime_now();
                Pose.x = msg.x;
                Pose.y = -msg.z; //optitrack z is world -y
                double roll;
                double pitch;
                double yaw;
                toEulerAngle(msg, roll, pitch, yaw);
                Pose.theta = yaw;
                BBmsg.pose = Pose;
            }            
        }
        BBmsg.utime = utime_now();
        lcmInstance.publish(OPTITRACK_CHANNEL, &BBmsg);
    }
    
    // Cleanup options now that we've parsed everything we need
    getopt_destroy(gopt);

    return 0;
}
