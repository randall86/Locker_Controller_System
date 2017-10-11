#include <DTIOI2CtoParallelConverter.h>
#include <SimpleModbusSlave.h>
#include <SimpleTimer.h>

//#define HW_TEST

#define RS485_EN_PIN 8 //RS485 Direction control

//address for product identification
#define ADDRBIT0_PIN 2
#define ADDRBIT1_PIN 3
#define ADDRBIT2_PIN 4
#define ADDRBIT3_PIN 5

//timer setting pins
#define TIMERBIT0_PIN 6
#define TIMERBIT1_PIN 7

//U5 expander pins
#define MAG_DOOR_PIN PIN1_4
#define ALARM_PIN PIN1_5
#define DOOR_SW_PIN PIN1_6
#define SPARE_PIN PIN1_7

#define CONV_RATE 60000 //60*1000 - mins to milisec

//Modbus Registers Offsets (0-9999)
enum 
{     
    // just add or remove registers and your good to go...
    // The first register starts at address 0
    LED_60 = 59, //LED starts from 0 - 59
    MAG_DOOR,
    ALARM,
    DOOR_SW,
    SPARE_REG,
    ADDR_0,
    ADDR_1,
    ADDR_2,
    ADDR_3,
    TIM_0,
    TIM_1,
    TEST_MODE,
    TOTAL_ERRORS,
    // leave this one
    TOTAL_REGS_SIZE 
    // total number of registers for function 3 and 16 share the same register array
};

typedef struct _reg_map_io_expandr_t
{
    DTIOI2CtoParallelConverter *expandr;
    byte expandr_bus;
    byte expandr_pin;
    byte expandr_pin_state;
}reg_map_io_expandr_t;

unsigned int holdingRegs[TOTAL_REGS_SIZE];
reg_map_io_expandr_t g_LEDMappingArr[LED_60+1];

DTIOI2CtoParallelConverter g_ioExpandrU2(0x74);  //PCA9539 I/O Expander (with A1 = 0 and A0 = 0)
DTIOI2CtoParallelConverter g_ioExpandrU3(0x75);  //PCA9539 I/O Expander (with A1 = 0 and A0 = 1)
DTIOI2CtoParallelConverter g_ioExpandrU4(0x76);  //PCA9539 I/O Expander (with A1 = 1 and A0 = 0)
DTIOI2CtoParallelConverter g_ioExpandrU5(0x77);  //PCA9539 I/O Expander (with A1 = 1 and A0 = 1)

//SimpleTimer object
SimpleTimer timer;

int g_LEDcount = 0;
int g_timeout_value = 0; //0 - timer disabled
int g_lock_timer = 0;
int g_alarm_timer = 0;
bool g_door_is_opened = false;
bool g_unlock_door = false;
bool g_tested_once = false;

void initLEDsOnExpandr(DTIOI2CtoParallelConverter *io_expandr)
{
    //init LEDs on P0x bus
    for(int P0_count = PIN0_0; ((P0_count <= PIN0_7) && (g_LEDcount <= LED_60)); P0_count++)
    {
        io_expandr->pinMode0(P0_count, LOW);
        io_expandr->digitalWrite0(P0_count, HIGH);        
        
        g_LEDMappingArr[g_LEDcount].expandr = io_expandr;
        g_LEDMappingArr[g_LEDcount].expandr_bus = 0;
        g_LEDMappingArr[g_LEDcount].expandr_pin = P0_count;
        g_LEDcount++;
    }

    //init LEDs on P1x bus
    for(int P1_count = PIN1_0; ((P1_count <= PIN1_7) && (g_LEDcount <= LED_60)); P1_count++)
    {
        io_expandr->pinMode1(P1_count, LOW);
        io_expandr->digitalWrite1(P1_count, HIGH);

        g_LEDMappingArr[g_LEDcount].expandr = io_expandr;
        g_LEDMappingArr[g_LEDcount].expandr_bus = 1;
        g_LEDMappingArr[g_LEDcount].expandr_pin = P1_count;
        g_LEDcount++;
    }
}

//compute the Modbus slave ID from the device product identification pins
unsigned char getSlaveAddr()
{
    unsigned char mask = 0x01;
    unsigned char bit00 = (mask & !digitalRead(ADDRBIT0_PIN));
    unsigned char bit01 = (mask & !digitalRead(ADDRBIT1_PIN));
    unsigned char bit02 = (mask & !digitalRead(ADDRBIT2_PIN));
    unsigned char bit03 = (mask & !digitalRead(ADDRBIT3_PIN));

    return ((bit03 << 3) | (bit02 << 2) | (bit01 << 1) | bit00);
}

//get the timer's timeout value from the timer setting pins
int getTimerValue()
{
    unsigned char mask = 0x01;
    unsigned char bit00 = (mask & !digitalRead(TIMERBIT0_PIN));
    unsigned char bit01 = (mask & !digitalRead(TIMERBIT1_PIN));
    
    return ((bit01 << 1) | bit00);
}

void turnOnOffAllLEDs()
{
    //turn on all the LEDS
    for(int num_LED = 0; num_LED <= LED_60; num_LED++)
    {
        if(g_LEDMappingArr[num_LED].expandr_bus == 0)
        {
            g_LEDMappingArr[num_LED].expandr->digitalWrite0(g_LEDMappingArr[num_LED].expandr_pin, LOW); 
        }
        else // g_LEDMappingArr[num_LED].expandr_bus == 1
        {
            g_LEDMappingArr[num_LED].expandr->digitalWrite1(g_LEDMappingArr[num_LED].expandr_pin, LOW);
        }
        
        delay(500);
    }
    
    //turn off all the LEDS
    for(int num_LED = 0; num_LED <= LED_60; num_LED++)
    {
        if(g_LEDMappingArr[num_LED].expandr_bus == 0)
        {
            g_LEDMappingArr[num_LED].expandr->digitalWrite0(g_LEDMappingArr[num_LED].expandr_pin, HIGH); 
        }
        else // g_LEDMappingArr[num_LED].expandr_bus == 1
        {
            g_LEDMappingArr[num_LED].expandr->digitalWrite1(g_LEDMappingArr[num_LED].expandr_pin, HIGH);
        }
        
        delay(500);
    }
}

void turnOffAllLEDs()
{
    for(int num_LED = 0; num_LED <= LED_60; num_LED++)
    {
        if(g_LEDMappingArr[num_LED].expandr_bus == 0)
        {
            g_LEDMappingArr[num_LED].expandr->digitalWrite0(g_LEDMappingArr[num_LED].expandr_pin, HIGH); 
        }
        else // g_LEDMappingArr[num_LED].expandr_bus == 1
        {
            g_LEDMappingArr[num_LED].expandr->digitalWrite1(g_LEDMappingArr[num_LED].expandr_pin, HIGH);
        }

        holdingRegs[num_LED] = LOW; //reset the Mobbus register state
    }
}

void timeoutAlarmRoutine()
{
    //start the alarm if the door is still open
    if(g_door_is_opened)
    {
        g_ioExpandrU5.digitalWrite1(ALARM_PIN, HIGH);
    }
}

void timeoutLockRoutine()
{
    //lock the door if its already closed
    if(!g_door_is_opened)
    {
        g_ioExpandrU5.digitalWrite1(MAG_DOOR_PIN, LOW);
        g_unlock_door = false;
        turnOffAllLEDs();
    }
}

void setup()
{
    Wire.begin(); //need to start the Wire for I2C devices to function
    
    //initialize the addr pins
    pinMode(ADDRBIT0_PIN, INPUT);
    pinMode(ADDRBIT1_PIN, INPUT);
    pinMode(ADDRBIT2_PIN, INPUT);
    pinMode(ADDRBIT3_PIN, INPUT);
    
    //initialize the timer pins
    pinMode(TIMERBIT0_PIN, INPUT);
    pinMode(TIMERBIT1_PIN, INPUT);
    
    //initialize the LED pins
    initLEDsOnExpandr(&g_ioExpandrU2);
    initLEDsOnExpandr(&g_ioExpandrU3);
    initLEDsOnExpandr(&g_ioExpandrU4);
    initLEDsOnExpandr(&g_ioExpandrU5);
    
    //initialize the MAG_DOOR - solenoid lock
    g_ioExpandrU5.pinMode1(MAG_DOOR_PIN, LOW);
    g_ioExpandrU5.digitalWrite1(MAG_DOOR_PIN, LOW);
    
    //initialize the ALARM
    g_ioExpandrU5.pinMode1(ALARM_PIN, LOW);
    g_ioExpandrU5.digitalWrite1(ALARM_PIN, LOW);
    
    //initialize the DOOR_SW
    g_ioExpandrU5.pinMode1(DOOR_SW_PIN, HIGH);
    
    //initialize the SPARE
    g_ioExpandrU5.pinMode1(SPARE_PIN, LOW);
    g_ioExpandrU5.digitalWrite1(SPARE_PIN, LOW);
    
    //get the timer's timeout value
    g_timeout_value = getTimerValue();
    
    // parameters(long baudrate, unsigned char ID, unsigned char transmit enable pin, unsigned int holding registers size, unsigned char low latency)
    // The transmit enable pin is used in half duplex communication to activate a MAX485 or similar
    // to deactivate this mode use any value < 2 because 0 & 1 is reserved for Rx & Tx.
    // Low latency delays makes the implementation non-standard
    // but practically it works with all major modbus master implementations.
#ifdef HW_TEST
    modbus_configure(115200, 1, RS485_EN_PIN, TOTAL_REGS_SIZE, 0);
#else
    modbus_configure(115200, getSlaveAddr(), RS485_EN_PIN, TOTAL_REGS_SIZE, 0);
#endif
}

void loop()
{
    // modbus_update() is the only method used in loop(). It returns the total error
    // count since the slave started. You don't have to use it but it's useful
    // for fault finding by the modbus master.
    holdingRegs[TOTAL_ERRORS] = modbus_update(holdingRegs);
    
    // this is where the "polling" occurs
    timer.run();
    
#ifdef HW_TEST
    byte door_switch = DOOR_SW_PIN;
    if(g_ioExpandrU5.digitalRead1(door_switch))
    {
        //attach Door switch pin to its Modbus register
        holdingRegs[DOOR_SW] = door_switch;
        
        if(door_switch)
        {
            //toggle the LEDs
            for(int num_LED = 0; num_LED <= LED_60; num_LED++)
            {
                if(g_LEDMappingArr[num_LED].expandr_bus == 0)
                {
                    g_LEDMappingArr[num_LED].expandr->digitalWrite0(g_LEDMappingArr[num_LED].expandr_pin, (HIGH - holdingRegs[num_LED]));
                }
                else
                {
                    g_LEDMappingArr[num_LED].expandr->digitalWrite1(g_LEDMappingArr[num_LED].expandr_pin, (HIGH - holdingRegs[num_LED]));
                }
            }
        }
        else
        {
            //toggle the LEDs based on the jumpers
            g_ioExpandrU2.digitalWrite0(PIN0_0, digitalRead(ADDRBIT0_PIN));
            g_ioExpandrU2.digitalWrite0(PIN0_1, digitalRead(ADDRBIT1_PIN));
            g_ioExpandrU2.digitalWrite0(PIN0_2, digitalRead(ADDRBIT2_PIN));
            g_ioExpandrU2.digitalWrite0(PIN0_3, digitalRead(ADDRBIT3_PIN));
            g_ioExpandrU2.digitalWrite0(PIN0_4, digitalRead(TIMERBIT0_PIN));
            g_ioExpandrU2.digitalWrite0(PIN0_5, digitalRead(TIMERBIT1_PIN));
        }
        
        //toggle the relays
        g_ioExpandrU5.digitalWrite1(MAG_DOOR_PIN, door_switch);
        g_ioExpandrU5.digitalWrite1(ALARM_PIN, door_switch);
        g_ioExpandrU5.digitalWrite1(SPARE_PIN, door_switch);
    }
#else
    if(holdingRegs[TEST_MODE]) //run test mode
    {
        if(!g_tested_once)
        {
            turnOnOffAllLEDs(); //test on/off all LEDs sequentially
            g_tested_once = true;
        }
        
        //toggle the LEDs according to the Modbus register values
        for(int num_LED = 0; num_LED <= LED_60; num_LED++)
        {
            if(g_LEDMappingArr[num_LED].expandr_bus == 0)
            {
                g_LEDMappingArr[num_LED].expandr->digitalWrite0(g_LEDMappingArr[num_LED].expandr_pin, (HIGH - holdingRegs[num_LED])); 
            }
            else // g_LEDMappingArr[num_LED].expandr_bus == 1
            {
                g_LEDMappingArr[num_LED].expandr->digitalWrite1(g_LEDMappingArr[num_LED].expandr_pin, (HIGH - holdingRegs[num_LED]));
            }
        }
        
        //toggle the Magnetic Door lock according its Modbus register value
        g_ioExpandrU5.digitalWrite1(MAG_DOOR_PIN, holdingRegs[MAG_DOOR]);
        
        //toggle the Alarm/Buzzer according its Modbus register value
        g_ioExpandrU5.digitalWrite1(ALARM_PIN, holdingRegs[ALARM]);
        
        //toggle the Spare relay according its Modbus register value
        g_ioExpandrU5.digitalWrite1(SPARE_PIN, holdingRegs[SPARE_REG]);
        
        byte door_switch = DOOR_SW_PIN;
        if(g_ioExpandrU5.digitalRead1(door_switch))
        {
            //attach Door switch pin to its Modbus register
            holdingRegs[DOOR_SW] = door_switch;
        }
        
        //attach the address and timer jumper state to its Modbus register
        holdingRegs[ADDR_0] = digitalRead(ADDRBIT0_PIN);
        holdingRegs[ADDR_1] = digitalRead(ADDRBIT1_PIN);
        holdingRegs[ADDR_2] = digitalRead(ADDRBIT2_PIN);
        holdingRegs[ADDR_3] = digitalRead(ADDRBIT3_PIN);
        holdingRegs[TIM_0] = digitalRead(TIMERBIT0_PIN);
        holdingRegs[TIM_1] = digitalRead(TIMERBIT1_PIN);
    }
    else //run normal mode
    {
        g_tested_once = false; //reset the LED tests
        
        if(!g_unlock_door) //check for LEDs state if door is locked
        {
            //attach the LED pins to the Modbus registers
            for(int num_LED = 0; num_LED <= LED_60; num_LED++)
            {
                byte led_state = (HIGH - holdingRegs[num_LED]); //invert the value
                
                //toggle the LEDs according to the Modbus register values
                if(g_LEDMappingArr[num_LED].expandr_bus == 0)
                {
                    g_LEDMappingArr[num_LED].expandr->digitalWrite0(g_LEDMappingArr[num_LED].expandr_pin, led_state); 
                }
                else // g_LEDMappingArr[num_LED].expandr_bus == 1
                {
                    g_LEDMappingArr[num_LED].expandr->digitalWrite1(g_LEDMappingArr[num_LED].expandr_pin, led_state);
                }
                
                if(!led_state)
                {
                    g_unlock_door = true;
                    holdingRegs[num_LED] = LOW; //reset the Mobbus register state
                }
            }
            
            //unlock the magnetic door and start the alarm if any LEDs are on
            if(g_unlock_door)
            {
                g_ioExpandrU5.digitalWrite1(MAG_DOOR_PIN, HIGH);
                g_ioExpandrU5.digitalWrite1(ALARM_PIN, HIGH);
            }
        }
        
        if(g_unlock_door) //check the door switch for its state only if door is unlock
        {
            byte door_switch = DOOR_SW_PIN;
            if(g_ioExpandrU5.digitalRead1(door_switch))
            {
                //attach Door switch pin to its Modbus register
                holdingRegs[DOOR_SW] = door_switch;
                
                if(door_switch) //door is opened
                {
                    if(!g_door_is_opened)
                    {
                        g_door_is_opened = true;
                        g_ioExpandrU5.digitalWrite1(ALARM_PIN, LOW);
                        
                        if(g_timeout_value) //if timer is enabled
                        {
                            timer.deleteTimer(g_alarm_timer); //delete the previous set timer
                            g_alarm_timer = timer.setTimeout((g_timeout_value * CONV_RATE), timeoutAlarmRoutine);
                        }
                    }
                }
                else //door is closed
                {
                    if(g_door_is_opened)
                    {
                        g_door_is_opened = false;
                        g_ioExpandrU5.digitalWrite1(ALARM_PIN, LOW);
                        
                        if(g_timeout_value) //if timer is enabled
                        {
                            timer.deleteTimer(g_lock_timer); //delete the previous set timer
                            g_lock_timer = timer.setTimeout((g_timeout_value * CONV_RATE), timeoutLockRoutine);
                        }
                    }
                    
                    //user acknowledged - toggle the Magnetic Door lock according its Modbus register value
                    if(holdingRegs[MAG_DOOR])
                    {
                        delay(1000); //add some delay to ensure door is closed properly before locking
                        g_ioExpandrU5.digitalWrite1(MAG_DOOR_PIN, LOW);
                        g_unlock_door = false;
                        holdingRegs[MAG_DOOR] = LOW; //reset the Mobbus register state
                        turnOffAllLEDs();
                    }
                }
            }
        }
    }
#endif
}

