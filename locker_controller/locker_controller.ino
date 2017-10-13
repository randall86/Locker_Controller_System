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
#define OPEN_MSEC 100 //stable time before registering open
#define CLOSE_MSEC 500 //stable time before registering close
#define CHECK_MSEC 50 //read door switch every 50ms when door is unlocked

enum _Locker_state_t
{
    LOCKED,
    UNLOCKED,
    OPENED,
    CLOSED,
    MAINTENANCE
};

//Modbus Registers Offsets (0-9999)
enum _Modbus_reg_t
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
int g_lock_timer = -1;
int g_alarm_timer = -1;
int g_debounce_timer = -1;

int my_state = LOCKED; //initial state
int prev_state = LOCKED; //initial state
byte g_debouncedDoorState = 0; //closed
unsigned char my_address = 0;

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
        
        delay(300);
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
        
        delay(300);
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

        holdingRegs[num_LED] = LOW; //reset the Modbus register state
    }
}

void timeoutAlarmRoutine()
{
    //start the alarm if the door is still open
    if(OPENED == my_state)
    {
        g_ioExpandrU5.digitalWrite1(ALARM_PIN, HIGH);
    }
}

void timeoutLockRoutine()
{
    //lock the door if its already closed
    if(CLOSED == my_state)
    {
        g_ioExpandrU5.digitalWrite1(MAG_DOOR_PIN, LOW);
        turnOffAllLEDs();

        //disables the door switch debounce service routine timer
        timer.disable(g_debounce_timer);
        
        my_state = LOCKED;
    }
}

byte readDoorSwitch()
{
    byte ret = 0;
    byte door_state = DOOR_SW_PIN;
    if(g_ioExpandrU5.digitalRead1(door_state))
    {
        ret = door_state;
    }

    return ret;
}

//returns true if state changed
bool debounceDoorSwitch(byte *state)
{
    static uint8_t count = OPEN_MSEC/CHECK_MSEC;
    bool state_changed = false;

    //read the door switch from the HW
    byte raw_state = readDoorSwitch();
    *state = g_debouncedDoorState;

    if (raw_state == g_debouncedDoorState)
    {
        //set the timer which allows a change from current state.
        if(g_debouncedDoorState)
        {
            count = CLOSE_MSEC/CHECK_MSEC;
        }
        else
        {
            count = OPEN_MSEC/CHECK_MSEC;
        }
    }
    else
    {
        //state has changed - wait for new state to become stable.
        if (--count == 0)
        {
            // Timer expired - accept the change.
            g_debouncedDoorState = raw_state;
            state_changed = true;
            *state = g_debouncedDoorState;
            
            // And reset the timer.
            if(g_debouncedDoorState) //door is opened
            {
                count = CLOSE_MSEC/CHECK_MSEC;
            }
            else //door is closed
            {
                count = OPEN_MSEC/CHECK_MSEC;
            }
        }
    }

    return state_changed;
}

void debounceDoorSwitchRoutine()
{
    byte door_switch_state = 0;
    
    //if door state changed, update the state
    if(debounceDoorSwitch(&door_switch_state))
    {
        //attach Door switch pin to its Modbus register
        holdingRegs[DOOR_SW] = door_switch_state;
                
        if(door_switch_state)
        {
            my_state = OPENED;
        }
        else
        {
            my_state = CLOSED;
        }
    }
}

//returns true if LED is on
bool checkLEDState()
{
    bool ret = false;

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
            ret = true;
            holdingRegs[num_LED] = LOW; //reset the Modbus register state
        }
    }

    return ret;
}

void handleLockedState()
{
    if(holdingRegs[TEST_MODE]) //enter Maintenance
    {
        my_state = MAINTENANCE;
    }
    else
    {
        //unlock the magnetic door and start the alarm if any LEDs are on
        if(checkLEDState())
        {
            g_ioExpandrU5.digitalWrite1(MAG_DOOR_PIN, HIGH);
            g_ioExpandrU5.digitalWrite1(ALARM_PIN, HIGH);
            my_state = UNLOCKED;
        }
    }

    prev_state = LOCKED;
}

void handleUnlockedState()
{
    if(UNLOCKED != prev_state)
    {
        //enables the debounce service routine for the door state
        timer.enable(g_debounce_timer);
    }
    
    prev_state = UNLOCKED;
}

void handleOpenedState()
{
    if(OPENED != prev_state)
    {
        //turn off the alarm once user opens the door
        g_ioExpandrU5.digitalWrite1(ALARM_PIN, LOW);
        
        //if timer is enabled and there is state changed
        if(g_timeout_value) 
        {
            timer.deleteTimer(g_alarm_timer); //delete the previous set timer
            g_alarm_timer = timer.setTimeout((g_timeout_value * CONV_RATE), timeoutAlarmRoutine);
        }
    }

    prev_state = OPENED;
}

void handleClosedState()
{
    if(CLOSED != prev_state)
    {
        //turn off the alarm once user closes the door
        g_ioExpandrU5.digitalWrite1(ALARM_PIN, LOW);
        
        //if timer is enabled and there is state changed
        if(g_timeout_value)
        {
            timer.deleteTimer(g_lock_timer); //delete the previous set timer
            g_lock_timer = timer.setTimeout((g_timeout_value * CONV_RATE), timeoutLockRoutine);
        }
    }
    
    //user acknowledged - toggle the Magnetic Door lock according its Modbus register value
    if(holdingRegs[MAG_DOOR])
    {
        holdingRegs[MAG_DOOR] = LOW; //reset the Modbus register state
        
        g_ioExpandrU5.digitalWrite1(MAG_DOOR_PIN, LOW);
        turnOffAllLEDs();

        //disables the door switch debounce service routine timer
        timer.disable(g_debounce_timer);

        my_state = LOCKED;
    }
    
    prev_state = CLOSED;
}

void doMaintenance()
{
    if(MAINTENANCE != prev_state)
    {
        turnOnOffAllLEDs(); //test on/off all LEDs sequentially
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

    if(!holdingRegs[TEST_MODE]) //exit Maintenance
    {
        my_state = LOCKED;
    }

    prev_state = MAINTENANCE;
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
    
    if(0 != g_timeout_value) //if timer is enabled
    {
        g_timeout_value += 2;  //add 2 here to get 3/4/5 as the jumper settings are only 1/2/3
    }
    
    // parameters(long baudrate, unsigned char ID, unsigned char transmit enable pin, unsigned int holding registers size, unsigned char low latency)
    // The transmit enable pin is used in half duplex communication to activate a MAX485 or similar
    // to deactivate this mode use any value < 2 because 0 & 1 is reserved for Rx & Tx.
    // Low latency delays makes the implementation non-standard
    // but practically it works with all major modbus master implementations.
#ifdef HW_TEST
    modbus_configure(115200, 1, RS485_EN_PIN, TOTAL_REGS_SIZE, 0);
#else
    my_address = getSlaveAddr();
    if(0 != my_address)
    {
        modbus_configure(115200, my_address, RS485_EN_PIN, TOTAL_REGS_SIZE, 0);
    }
#endif

    //initialize the debounce timer
    g_debounce_timer = timer.setInterval(CHECK_MSEC, debounceDoorSwitchRoutine);
    timer.disable(g_debounce_timer);
}

void loop()
{    
#ifdef HW_TEST
    // modbus_update() is the only method used in loop(). It returns the total error
    // count since the slave started. You don't have to use it but it's useful
    // for fault finding by the modbus master.
    holdingRegs[TOTAL_ERRORS] = modbus_update(holdingRegs);
    
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
    if(0 != my_address) //only run logic if a valid device address is used
    {
        // modbus_update() is the only method used in loop(). It returns the total error
        // count since the slave started. You don't have to use it but it's useful
        // for fault finding by the modbus master.
        holdingRegs[TOTAL_ERRORS] = modbus_update(holdingRegs);
        
        // this is where the "polling" occurs
        timer.run();
        
        switch(my_state)
        {
            case LOCKED:
                handleLockedState();
                break;
                
            case UNLOCKED:
                handleUnlockedState();
                break;
                
            case OPENED:
                handleOpenedState();
                break;
                
            case CLOSED:
                handleClosedState();
                break;
                
            case MAINTENANCE:
                doMaintenance();
                break;
        }
    }
#endif
}

