static uint16_t channels[8] = {1500,1500,1000,1500,1000,1000,1000,1000};
static float plant_height=60.0f, plant_velocity;
static int tick_number;

static void send_rc(int corrupt)
{
    unsigned i;
    memset(mock_packet,0,32);
    mock_packet[0]=0xAA; mock_packet[1]=0xAF;
    for (i=0;i<8;i++) {
        mock_packet[4+i*2]=(uint8_t)(channels[i]>>8);
        mock_packet[5+i*2]=(uint8_t)channels[i];
    }
    for (i=0;i<31;i++) mock_packet[31]+=mock_packet[i];
    if (corrupt) ++mock_packet[31];
    packet_available=1;
    RC_Analy();
}
static void sample_height(float mm)
{
    unsigned raw=(unsigned)(mm/FLOW_HEIGHT_MM_PER_UNIT+0.5f);
    mock_sensor[0]=(uint8_t)(raw>>8); mock_sensor[1]=(uint8_t)raw;
}
static void step(float mm, int valid_rc)
{
    SysTick_count+=2;
    if (valid_rc) send_rc(0);
    sample_height(mm);
    flow_get_data();
    AutoFlight_Tick(0.006f);
    HeightPidControl(0.006f);
    if ((++tick_number%4)==0) { flow_data_sins(); FlowPidControl(0.024f); }
    FlightPidControl(0.003f); MotorControl();
    FlightPidControl(0.003f); MotorControl();
    PilotLED();
}
static void setup(void)
{
    pidHeightHigh.kp=1.2f; pidHeightHigh.kd=0.085f;
    pidHeightRate.kp=1.2f; pidHeightRate.ki=0.04f; pidHeightRate.kd=0.085f;
    pidPosRateX.kp=pidPosRateY.kp=0.15f;
    pidPosRateX.ki=pidPosRateY.ki=0.04f;
    pidPositionX.kp=pidPositionY.kp=0.02f;
    mock_sensor[4]=50;
    AutoFlight_Init();
}
static void prepare_ground(void)
{
    int i;
    for (i=0;i<150;i++) step(60.0f,1);
    assert(!ALL_flag.unlock);
}
static void press(void)
{
    int i;
    channels[6] = channels[6]>1500?1000:2000;
    for (i=0;i<12;i++) step(60.0f,1);
}
static void launch(void)
{
    prepare_ground(); press();
    assert(ALL_flag.unlock && AutoFlight_Active());
    assert(auto_flight.state==AUTO_SPOOL);
}
static void forced_hover(void)
{
    int i;
    launch();
    for(i=0;i<1200;i++) step(i<140?60.0f:800.0f,1);
    assert(auto_flight.state==AUTO_HOVER);
    assert(fabsf(auto_flight.setpoint_mm-800.0f)<0.01f);
    assert(ALL_flag.height_lock && ALL_flag.flow_control);
}
static void plant_step(float hover_pwm)
{
    float thrust=ALL_flag.unlock?auto_flight.throttle:0.0f;
    float acceleration=6.0f*(thrust-hover_pwm)-1.6f*plant_velocity;
    plant_velocity+=acceleration*0.006f;
    plant_height+=plant_velocity*0.006f;
    if (plant_height<60.0f) { plant_height=60.0f; plant_velocity=0.0f; }
    step(plant_height,1);
}
int main(int argc,char **argv)
{
    int i; const char *name;
    assert(argc==2); name=argv[1]; setup();
    if (!strcmp(name,"broken_sensor")) {
        for(i=0;i<200;i++) step(0.0f,1);
        press();
        assert(!ALL_flag.unlock && !AutoFlight_Active());
        assert(auto_flight.error==AUTO_REJECT_HEIGHT);
        assert(TIM2->CCR1==0 && TIM2->CCR4==0);
    } else if (!strcmp(name,"button")) {
        /* A high AUX3 present at startup is a baseline, never a command. */
        channels[6]=2000;
        prepare_ground();
        assert(!AutoFlight_Active());
        channels[6]=1000;
        for(i=0;i<20;i++) step(60.0f,1);
        if (AUTO_AUX3_LATCHED) {
            assert(AutoFlight_Active());
        } else {
            assert(!AutoFlight_Active()); press();
        }
        for(i=0;i<200;i++) step(i<130?60.0f:300.0f,1);
        assert(ALL_flag.unlock && auto_flight.state==AUTO_CLIMB);
        assert(auto_flight.throttle>0.0f && TIM2->CCR1>0);
        assert(channels[2]==1000); /* Low RC throttle cannot force descent. */
        channels[6]=channels[6]>1500?1000:2000;
        for(i=0;i<20;i++) step(300.0f,1);
        assert(auto_flight.state==AUTO_CLIMB);
    } else if (!strcmp(name,"hover_model")) {
        launch();
        for(i=0;i<4000;i++) {
            plant_step(520.0f);
            assert(auto_flight.throttle>=0 && auto_flight.throttle<=AUTO_MAX_THROTTLE);
        }
        printf("Toy plant final height %.1f mm, speed %.1f mm/s; ",plant_height,plant_velocity);
        assert(auto_flight.state==AUTO_HOVER);
        assert(fabsf(plant_height-800.0f)<200.0f);
        assert(fabsf(auto_flight.setpoint_mm-800.0f)<0.01f);
    } else if (!strcmp(name,"zero_and_fault")) {
        uint32_t old_tick; float throttle;
        forced_hover(); old_tick=flow_last_valid_tick; throttle=auto_flight.throttle;
        for(i=0;i<15;i++) step(0.0f,1);
        assert(!flow_height_valid && flow_last_valid_tick==old_tick);
        assert(auto_flight.stable_seconds==0.0f);
        assert(fabsf(auto_flight.throttle-throttle)<0.001f);
        for(i=0;i<10;i++) step(0.0f,1);
        assert(auto_flight.state==AUTO_RANGE_FAULT && ALL_flag.unlock);
        assert(!ALL_flag.height_lock && !ALL_flag.flow_control);
        assert(fabsf(auto_flight.throttle-throttle)<0.001f);
        for(i=0;i<100;i++) step(800.0f,1);
        assert(auto_flight.state==AUTO_RANGE_FAULT); /* No silent auto-restart. */
        channels[3]=1000; step(800.0f,1);
        assert(!ALL_flag.unlock && TIM2->CCR1==0);
    } else if (!strcmp(name,"alarms")) {
        forced_hover();
        for(i=0;i<180;i++) step(550.0f,1);
        assert(auto_flight.alarm==AUTO_ALARM_LOW);
        for(i=0;i<80;i++) {
            step(550.0f,1);
            assert(TIM3->CCR1==1000 && TIM3->CCR2==1000);
            assert(TIM3->CCR3==TIM3->CCR4);
            assert(TIM3->CCR3==(((SysTick_count/67U)&1U)?500U:1000U));
        }
        for(i=0;i<200;i++) step(1050.0f,1);
        assert(auto_flight.alarm==AUTO_ALARM_HIGH);
        for(i=0;i<80;i++) {
            step(1050.0f,1);
            assert(TIM3->CCR3==1000 && TIM3->CCR4==1000);
            assert(TIM3->CCR1==TIM3->CCR2);
        }
        for(i=0;i<200;i++) step(800.0f,1);
        assert(auto_flight.alarm==AUTO_ALARM_NONE);
    } else if (!strcmp(name,"landing_model")) {
        launch();
        for(i=0;i<3000;i++) plant_step(500.0f);
        assert(auto_flight.state==AUTO_HOVER);
        channels[2]=1500;
        for(i=0;i<50;i++) plant_step(500.0f);
        channels[2]=1000;
        for(i=0;i<5000 && ALL_flag.unlock;i++) plant_step(500.0f);
        assert(!ALL_flag.unlock && !AutoFlight_Active());
        assert(plant_height<100.0f && TIM2->CCR1==0 && TIM2->CCR4==0);
    } else if (!strcmp(name,"rc_timeout")) {
        forced_hover();
        for(i=0;i<510;i++) {
            SysTick_count+=2;
            send_rc(1); /* Corrupt traffic cannot refresh link freshness. */
            sample_height(800); flow_get_data();
            AutoFlight_Tick(0.006f); MotorControl();
        }
        assert(!ALL_flag.unlock && !AutoFlight_Active() && TIM2->CCR1==0);
    } else if (!strcmp(name,"takeover")) {
        float before;
        forced_hover(); before=auto_flight.throttle;
        channels[4]=2000; step(800,1);
        assert(auto_flight.takeover_pending && AutoFlight_Active());
        channels[2]=(uint16_t)(1000.0f+(before-200.0f)/0.45f);
        for(i=0;i<4;i++) step(800,1);
        assert(!AutoFlight_Active() && ALL_flag.unlock && !ALL_flag.height_lock);
        assert(fabsf((float)TIM2->CCR1-before)<45.0f);
    } else if (!strcmp(name,"tilt")) {
        launch();
        for(i=0;i<10;i++) step(60,1);
        Angle.roll=55.0f;
        FlightPidControl(0.003f); MotorControl();
        assert(!ALL_flag.unlock && TIM2->CCR1==0);
    } else if (!strcmp(name,"flow_loss")) {
        forced_hover(); mock_sensor[4]=0;
        for(i=0;i<360;i++) step(800,1);
        assert(auto_flight.state==AUTO_LANDING && auto_flight.error==AUTO_ERROR_FLOW);
    } else if (!strcmp(name,"reject_conditions")) {
        prepare_ground(); channels[2]=1500; press();
        assert(!ALL_flag.unlock && auto_flight.error==AUTO_REJECT_STICKS);
        channels[2]=1000; channels[6]=1000;
        for(i=0;i<150;i++) step(900.0f,1);
        channels[6]=2000;
        for(i=0;i<15;i++) step(900.0f,1);
        assert(!ALL_flag.unlock && auto_flight.error==AUTO_REJECT_GROUND);
    } else if (!strcmp(name,"manual_legacy")) {
        prepare_ground();
        channels[2]=1900;
        for(i=0;i<10;i++) step(60,1);
        channels[2]=1000;
        for(i=0;i<5;i++) step(60,1);
        assert(ALL_flag.unlock && !AutoFlight_Active());
        channels[2]=1500; for(i=0;i<5;i++) step(60,1);
        assert(TIM2->CCR1>0);
        channels[2]=1000; channels[3]=1000; step(60,1);
        assert(!ALL_flag.unlock && TIM2->CCR1==0);
    } else if (!strcmp(name,"signed_and_range")) {
        sample_height(800); mock_sensor[2]=0xFE; mock_sensor[3]=0x80;
        flow_get_data();
        assert(flow_data.vel_x==-2 && flow_data.vel_y==-128);
        assert(VL53L01_high==800 && flow_height_valid);
        sample_height(5000); flow_get_data();
        assert(!flow_height_valid && VL53L01_high==800 && flow_valid_samples==0);
        sample_height(800); mock_bus_fails=1; flow_get_data();
        assert(!flow_height_valid && flow_data.squal==0);
    } else if (!strcmp(name,"i2c_low_line")) {
        gpio_fixture.IDR=1U; /* Another pin is high, but SDA/PB7 is stuck low. */
        assert(!SDA_read);
        gpio_fixture.IDR=129U;
        assert(!(!SDA_read));
        gpio_fixture.IDR=65U;
        assert(!(!SCL_read));
    } else if (!strcmp(name,"alarm_boundaries")) {
        forced_hover();
        for(i=0;i<200;i++) step(600.0f,1);
        assert(auto_flight.alarm==AUTO_ALARM_NONE);
        for(i=0;i<200;i++) step(800.0f,1);
        for(i=0;i<200;i++) step(1000.0f,1);
        assert(auto_flight.alarm==AUTO_ALARM_NONE);
            for(i=0;i<200;i++) step(650.0f,1);
        assert(auto_flight.alarm==AUTO_ALARM_NONE);
        for(i=0;i<200;i++) step(950.0f,1);
        assert(auto_flight.alarm==AUTO_ALARM_NONE);
    } else if (!strcmp(name,"ground_sensor_loss")) {
        launch();
        for(i=0;i<150;i++) step(60.0f,1);
        assert(auto_flight.state==AUTO_CLIMB);
        for(i=0;i<25;i++) step(0.0f,1);
        assert(!ALL_flag.unlock && !AutoFlight_Active());
        assert(auto_flight.error==AUTO_ERROR_RANGE && TIM2->CCR1==0);
    } else assert(!"Unknown test scenario");
    puts("passed"); return 0;
}
