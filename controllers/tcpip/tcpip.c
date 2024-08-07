/*
 * Copyright 1996-2023 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:  A simple program implementing a TCP/IP relay controller for
 *               interfacing Webots with any development environment able to
 *               use TCP/IP, including MathLab, Lisp, Java, C, C++, etc.
 * Author:       Darren Smith
 */

/* The protocole used in this example is taken from the Khepera serial
 * communication protocole. Hence, if you already have developed an
 * application which uses this protocole (and send the data over the serial
 * port), you will just need to redirect the data to the TCP/IP socket of
 * this controller to make it work with Webots.
 *
 * Currently supported Khepera protocole commands include:
 *
 * B: read software version
 * D: set speed
 * G: set position counter
 * H: read position
 * L: change LED state
 * N: read proximity sensors
 * O: read ambient light sensors
 *
 * A sample client program, written in C is included in this directory.
 * See client.c for the source code
 * compile it with gcc client.c -o client
 *
 * Everything relies on standard POSIX TCP/IP sockets.
 */

#include <math.h>
#include <stdio.h>
#include <string.h>
#include <sys/types.h>
#include <time.h>
#include <webots/distance_sensor.h>
#include <webots/led.h>
#include <webots/light_sensor.h>
#include <webots/motor.h>
#include <webots/position_sensor.h>
#include <webots/robot.h>


#ifdef _WIN32
#include <winsock.h>
#else
#include <arpa/inet.h>  /* definition of inet_ntoa */
#include <netdb.h>      /* definition of gethostbyname */
#include <netinet/in.h> /* definition of struct sockaddr_in */
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h> /* definition of close */
#endif

#define SOCKET_PORT 10019
#define TIMESTEP 250
#define JOINT_NUM 4


static WbDeviceTag position[JOINT_NUM];
static WbDeviceTag motor[JOINT_NUM];

static int fd;
static fd_set rfds;

static int accept_client(int server_fd) {
  int cfd;
  struct sockaddr_in client;
  int asize;
  struct hostent *client_info;
  asize = sizeof(struct sockaddr_in);

  cfd = accept(server_fd, (struct sockaddr *)&client, &asize);
  if (cfd == -1) {
    printf("cannot accept client\n");
    return -1;
  }
  client_info = gethostbyname((char *)inet_ntoa(client.sin_addr));
  printf("Accepted connection from: %s \n", client_info->h_name);

  return cfd;
}

static int create_socket_server(int port) {
  int sfd, rc;
  struct sockaddr_in address;

  /* create the socket */
  sfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sfd == -1) {
    printf("cannot create socket\n");
    return -1;
  }

  /* fill in socket address */
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons((unsigned short)port);
  address.sin_addr.s_addr = INADDR_ANY;

  /* bind to port */
  rc = bind(sfd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    printf("cannot bind port %d\n", port);
    close(sfd);
    return -1;
  }

  /* listen for connections */
  if (listen(sfd, 1) == -1) {
    printf("cannot listen for connections\n");
    close(sfd);
    return -1;
  }
  printf("Waiting for a connection on port %d...\n", port);

  return accept_client(sfd);
}







static void initialize() {
  int i;
  char text[32];

  for (i = 1; i < JOINT_NUM + 1; i++) {
    sprintf(text, "joint_%d_sensor", i);
    // printf("get %s\n",text);
    position[i-1] = wb_robot_get_device(text);
    wb_position_sensor_enable(position[i-1], TIMESTEP);
  }

  for (i = 1; i < JOINT_NUM + 1; i++) {
    sprintf(text, "joint_%d", i);
    // printf("get %s\n",text);
    motor[i-1] = wb_robot_get_device(text);
    wb_motor_set_position(motor[i-1], 0.0);
  }

  printf("robot has been initialized by Webots\n");
  fd = create_socket_server(SOCKET_PORT);
  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);
}

static void run() {
  int n;
  int ret;
  char buffer[256];

  int pos[4];

  struct timeval tv = {0, 0};
  int number;


  /* Set up the parameters used for the select statement */

  FD_ZERO(&rfds);
  FD_SET(fd, &rfds);

  /*
   * Watch TCPIP file descriptor to see when it has input.
   * No wait - polling as fast as possible
   */
  number = select(fd + 1, &rfds, NULL, NULL, &tv);

  /* If there is no data at the socket, then redo loop */
  if (number == 0)
    return;

  /* ...otherwise, there is data to read, so read & process. */
  n = recv(fd, buffer, 256, 0);
  if (n < 0) {
    printf("error reading from socket\n");
    return;
  }
  buffer[n] = '\0';
  printf("Received %d bytes: %s\n", n, buffer);

  if (buffer[0] == 'P') { /* set the speed of the motors */
    sscanf(buffer, "P,%d,%d,%d,%d", &pos[0], &pos[1], &pos[2], &pos[3]);
    for(int i = 0; i < JOINT_NUM; ++i){
      wb_motor_set_position(motor[i], pos[i]);
    }
    send(fd, "d\r\n", 3, 0);

  } else if (buffer[0] == 'L') {
   

  } else if (buffer[0] == 'G') { /* set the position counter */
    

  } else if (buffer[0] == 'B') { /* return a pretend version string */
    

  } else if (buffer[0] == 'N') { /* read distance sensor values */
    sprintf(buffer, "n,%d,%d,%d,%d\r\n",
    (int)wb_position_sensor_get_value(position[0]),
     (int)wb_position_sensor_get_value(position[1]),
    (int)wb_position_sensor_get_value(position[2]),
     (int)wb_position_sensor_get_value(position[3]));
            
    send(fd, buffer, strlen(buffer), 0);

  } else if (buffer[0] == 'H') {
    

  } else if (strncmp(buffer, "exit", 4) == 0) {
    printf("connection closed\n");
    ret = close(fd);
    if (ret != 0) {
      printf("Cannot close socket\n");
    }
    fd = 0;
  } else {
    send(fd, "\n", 1, 0);
  }
}

int main() {
  wb_robot_init();

  initialize();

  while (1) {
    wb_robot_step(TIMESTEP);
    run();
  }

  wb_robot_cleanup();

  return 0;
}
