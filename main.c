//Creators: Elias Valle, Van Vo, Elmo Vahvaselkä

#include <project.h>
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"
#include "Motor.h"
#include "Ultra.h"
#include "Nunchuk.h"
#include "Reflectance.h"
#include "Gyro.h"
#include "Accel_magnet.h"
#include "LSM303D.h"
#include "IR.h"
#include "Beep.h"
#include "mqtt_sender.h"
#include <time.h>
#include <sys/time.h>
#include "serial1.h"
#include <unistd.h>
#include <stdlib.h>

#define ON 1
#define OFF 0

#define BLACK 1
#define WHITE 0 

#define NORTH 0
#define EAST 1
#define SOUTH 2
#define WEST 3

#define TURN_LEFT -1
#define TURN_RIGHT 1

void progStart(uint32_t delay);
void progEnd(void); 
void motor_tankturn_right(uint8 speed, uint32 delay);
void motor_tankturn_left(uint8 speed, uint32 delay);
void motor_stop_next_line_end(uint8 speed, struct sensors_ dig);
void motor_follow_line(struct sensors_ dig);
void maze_turn_right(); 
void maze_turn_left();
void maze_check_wall_turn_left(struct sensors_ dig);
void maze_forward(struct sensors_ dig);
void update_direction(int turn);
void update_position();

bool turn_right;
bool wall_next_crossection;
int direction; 
int x, y;

//Maze
#if 1
void zmain (void){
    //Stating variables and initiating components
    struct sensors_ dig;
    turn_right = false;
    wall_next_crossection = false;
    direction = NORTH; 
    x = 0, y = -1; //Starting coordinates
    TickType_t start = 0;
    TickType_t stop = 0;    
    reflectance_start();
    reflectance_set_threshold(15000, 15000, 11000, 11000, 15000, 15000);
    motor_start();
    motor_forward(0,0);
    IR_Start();
    Ultra_Start();    
    
    progStart(1000);
    
    //Moving to the startline and waiting for IR signal
    motor_stop_next_line_end(250, dig);
    print_mqtt("Zumo07/ready", "maze");
    IR_wait();
    start = xTaskGetTickCount();
    print_mqtt("Zumo07/start", "%d", start);
    
    //Moving to center of the first complete line
    maze_forward(dig);
    
    //Main logic, for reaching the last complete line
    while(y < 11){ 
        if(x == 3 && direction == EAST){ //If east edge is reached, turn north             
            maze_turn_left(); 
        }else if(turn_right == true){  //Turn right if possible. The flag is set in maze_forward();
            maze_turn_right();          
        }
        while(Ultra_GetDistance() <= 13){ //Check for obstacle
            if(direction == SOUTH && x == 3){
                maze_turn_left();
                maze_turn_left();
            }else{            
                maze_check_wall_turn_left(dig); //Logic for circling around the obstacles 
            }
                
        }        
        turn_right = false; //Resets the flag from maze_forward
        maze_forward(dig); //Also prints the coordinates
        motor_forward(0,0); 
    } 
       
    //Driving to the center of the last complete line
    if(x < 0){ //If on the left side
        maze_turn_right();
        while(x < 0){
            maze_forward(dig);
        }
    }else if (x > 0){ //If on the right side
        maze_turn_left();
        while(x > 0){
            maze_forward(dig);
        }
    }
    
    //Driving to the goal
    if(direction == EAST){
        maze_turn_left();
    } else if(direction == WEST){
        maze_turn_right();
    }
    while(y < 13){  //For reaching last crossection
        maze_forward(dig);
    }
    reflectance_digital(&dig);
    while(dig.L1 == BLACK && dig.R1 == BLACK){ //For reaching the end of the line
        reflectance_digital(&dig);
        motor_forward(250,0);
    }
    motor_forward(0,0);
    stop = xTaskGetTickCount();
    print_mqtt("Zumo07/stop", "%d", stop);
    print_mqtt("Zumo07/time", "%d", stop-start);
    progEnd();                 
}    
#endif

//Line follower
#if 0
void zmain (void){
    //Stating variables and initiating components
    struct sensors_ dig;
    TickType_t start = 0;
    TickType_t stop = 0;
    IR_Start();
    reflectance_start();
    reflectance_set_threshold(15000, 15000, 11000, 11000, 15000, 15000);
    motor_start();
    motor_forward(0,0);
    
    progStart(1000);
    
    //Stop at the edge of the start line and wait for the IR signal
    motor_stop_next_line_end(250, dig); 
    print_mqtt("Zumo07/ready", "line");
    IR_wait();
    start = xTaskGetTickCount();
    print_mqtt("Zumo07/start", "%d", start);
    
    //Following the line until goal is reached
    while(true){
        reflectance_digital(&dig);
        if(dig.L3 == BLACK && dig.R3 == BLACK){ 
            while(dig.L3 == BLACK && dig.R3 == BLACK){ //Ignoring the first horizontal line 
                reflectance_digital(&dig);
            }
            motor_stop_next_line_end(250, dig); //Stopping at the goal line
            stop = xTaskGetTickCount();
            print_mqtt("Zumo07/stop", "%d", stop);
            break;            
        }
        reflectance_digital(&dig);
        motor_follow_line(dig); //The logic for line following and checking misses       
    }
    print_mqtt("Zumo07/time", "%d", stop-start);
    
    progEnd();
}        
#endif

//Sumo
#if 0
void zmain (void){
    //Stating variables and initiating components
    struct sensors_ dig;
    TickType_t start = 0;
    TickType_t stop = 0;
    IR_Start();
    Ultra_Start();    
    reflectance_start();
    reflectance_set_threshold(15000, 15000, 11000, 11000, 15000, 15000);
    motor_start();
    motor_forward(0,0);
    
    progStart(1000);
    
    //Stop at the edge of the ring and wait for the IR signal
    motor_stop_next_line_end(250, dig);
    print_mqtt("Zumo07/ready", "Zumo");
    IR_wait();
    start = xTaskGetTickCount();
    print_mqtt("Zumo07/start", "%d", start);
    motor_forward(200,100);
    
    //Moving inside the ring untill the button is pressed
    while(SW1_Read()){      
        reflectance_digital(&dig);
        if(dig.L3 == BLACK || dig.R3 == BLACK){     //Check the edge and turn if necessary
            motor_tankturn_left(200,100);
        }else if(Ultra_GetDistance() <= 10){    //Check for obstacles
            motor_tankturn_left(200,100);
            print_mqtt("Zumo07/obstacle", "%d", xTaskGetTickCount());
        }
        motor_forward(200,0);        
    }
    
    //Stop and print time
    motor_forward(0,0);
    stop = xTaskGetTickCount();
    print_mqtt("Zumo07/stop", "%d", stop);
    print_mqtt("Zumo07/time", "%d", stop-start);
    
    progEnd();
}    
#endif

//Custom functions
void progStart(uint32_t delay){
    while(SW1_Read() == 1);
    BatteryLed_Write(ON); 
    vTaskDelay(delay);
    BatteryLed_Write(OFF);     
}

void progEnd(void){
    bool led = 0;
    while(true){
        vTaskDelay(100);
        BatteryLed_Write(led^=1);
    }
}

void motor_tankturn_right(uint8 speed, uint32 delay){
    SetMotors(0, 1, speed, speed, delay);
}

void motor_tankturn_left(uint8 speed, uint32 delay){
    SetMotors(1, 0, speed, speed, delay);
}

void motor_stop_next_line_end(uint8 speed, struct sensors_ dig){
    motor_forward(speed,0);
    while(true){  
        reflectance_digital(&dig);  
        if(dig.L3 == BLACK && dig.R3 == BLACK){
            while(dig.L3 == BLACK && dig.R3 == BLACK){ 
                reflectance_digital(&dig);
            }
            motor_forward(0,0); 
            break;
        }
    }
}

void motor_follow_line(struct sensors_ dig){
    //Returning to line if missed and printing notifications
    if(dig.L1 == WHITE && dig.R1 == WHITE){ 
        print_mqtt("Zumo07/miss", "%d", xTaskGetTickCount());
        while(dig.L1 == WHITE && dig.R1 == WHITE){
            reflectance_digital(&dig);
            motor_backward(250,0);
        }
        print_mqtt("Zumo07/line", "%d", xTaskGetTickCount());
    
    //Forward if center on the line
    }else if(dig.L1 == BLACK && dig.R1 == BLACK){
        motor_forward(250,0);
        
    //Correcting direction if one center sensor misses the line
    }else if(dig.L1 == WHITE){ 
        motor_turn(250,0,0);
    }else if(dig.R1 == WHITE){
        motor_turn(0,250,0);
    }      
}

void maze_forward(struct sensors_ dig){    
    motor_forward(250,0);
    reflectance_digital(&dig);
    while(true){  
        reflectance_digital(&dig);
        if((dig.L3 == BLACK && dig.R3 == BLACK)||(dig.L3 == BLACK && dig.L2 == BLACK)||(dig.R3 == BLACK && dig.R2 == BLACK)){
            if(dig.R3 == BLACK && dig.R2 == BLACK){ //check if it's possible turn right
                turn_right = true;
            }
            while((dig.L3 == BLACK && dig.R3 == BLACK)||(dig.L3 == BLACK && dig.L2 == BLACK)||(dig.R3 == BLACK && dig.R2 == BLACK)){ 
                reflectance_digital(&dig); 
            }
            motor_forward(0,0);
            break;
        }
    }
    update_position();
    motor_forward(250,53);
    motor_forward(0,0);
}

void maze_turn_left(){
    motor_tankturn_left(50,524);
    motor_forward(0,0);
    update_direction(TURN_LEFT);
}

void maze_turn_right(){
    motor_tankturn_right(50,524);
    motor_forward(0,0);
    update_direction(TURN_RIGHT);
}

void maze_check_wall_turn_left(struct sensors_ dig){
    motor_tankturn_left(50,262); //45deg
    if(Ultra_GetDistance() <= 20){ //Check if the wall continues on the next crossection
        wall_next_crossection = true;
        motor_tankturn_left(50,262); //45deg
    }else{
        motor_tankturn_left(50,262); // 45deg
    }
    update_direction(TURN_LEFT);
    motor_forward(0,10); //small buffer, ensuring that robot sees certain walls.
    if(wall_next_crossection == false && y < 10 && !(Ultra_GetDistance() <= 15)){ //If wall ends next crossection, drives around the corner
        maze_forward(dig);
        maze_turn_right();
    }
    wall_next_crossection = false;
} 

void update_direction(int turn){ 
    if(turn == TURN_LEFT && direction == NORTH ){
        direction = WEST;
    }else{
        direction = (direction + turn)%4;
    }
}

void update_position(){
    if(direction == NORTH){
        y++;
    }
    if(direction == EAST){
        x++;
    }
    if(direction == SOUTH){
        y--;
    }
    if(direction == WEST){
        x--;
    }
    print_mqtt("Zumo07/position", "%d %d", x, y);
}