/* Included after the legacy flight fixture helpers, before main(). */
static void light_lux(float value)
{
    mock_light_raw = (uint16_t)(value * 1.2f + 0.5f);
}
static void light_steps(int count)
{
    int j;
    for (j=0;j<count;++j) step(800.0f,1);
}
static void light_ready(void)
{
    light_lux(300.0f);
    forced_hover();
    light_steps(240);
    assert(light_task.state==LIGHT_WAIT_RISE);
    assert(fabsf(light_task.ambient_lux-300.0f)<1.0f);
    assert(fabsf(light_task.arrival_lux-550.0f)<1.0f);
}
static void light_detect(void)
{
    light_ready();
    light_lux(400.0f);
    light_steps(100);
    assert(light_task.state==LIGHT_MANUAL_APPROACH);
    assert(light_task.source_detected && light_task.event_id==1);
    assert(LightTask_AlertActive());
}
static void light_enter_hold(void)
{
    int j;
    light_detect(); light_lux(600.0f);
    for(j=0;j<160 && !LightTask_RemoteLocked();++j) step(800,1);
    assert(LightTask_RemoteLocked());
    assert(light_task.stable_seconds==0.0f);
}
static void manual_light_ready(uint8_t position_hold)
{
    prepare_ground();
    ALL_flag.unlock = 1;
    channels[2] = 1500;
    channels[4] = position_hold ? 2000 : 1000;
    send_rc(0);
    ALL_flag.height_lock = ALL_flag.flow_control = position_hold;
    light_lux(300.0f);
    light_steps(400);
    assert(auto_flight.state == AUTO_IDLE);
    assert(light_task.state == LIGHT_WAIT_RISE);
}
static int run_light_case(const char *name)
{
    int j;
    if (!strcmp(name,"light_armed_ground_takeoff")) {
        prepare_ground(); ALL_flag.unlock = 1; press();
        assert(auto_flight.state == AUTO_SPOOL && ALL_flag.unlock);
    } else if (!strcmp(name,"light_armed_reject_height")) {
        prepare_ground(); ALL_flag.unlock = 1;
        for(j=0;j<20;++j) step(0,1);
        channels[6]=2000;
        for(j=0;j<12;++j) step(0,1);
        assert(auto_flight.state==AUTO_IDLE && auto_flight.error==AUTO_REJECT_HEIGHT);
        assert(auto_flight.start_request_id==1);
    } else if (!strcmp(name,"light_armed_airborne_reject")) {
        prepare_ground(); ALL_flag.unlock=1;
        for(j=0;j<200;++j) step(800,1);
        channels[6]=2000;
        for(j=0;j<12;++j) step(800,1);
        assert(auto_flight.state==AUTO_IDLE && auto_flight.error==AUTO_REJECT_GROUND);
    } else if (!strcmp(name,"light_manual_flight_alert")) {
        manual_light_ready(0); light_lux(400); light_steps(100);
        assert(light_task.source_detected && LightTask_AlertActive());
        assert(!AutoFlight_Active() && !LightTask_RemoteLocked());
        PilotLED(); assert(TIM3->CCR1==TIM3->CCR3);
    } else if (!strcmp(name,"light_manual_no_position_hold")) {
        manual_light_ready(0); light_lux(600); light_steps(200);
        assert(light_task.source_detected && !LightTask_RemoteLocked());
        assert(!ALL_flag.height_lock && !ALL_flag.flow_control);
    } else if (!strcmp(name,"light_manual_mode_hold")) {
        float yaw, hx, hy;
        manual_light_ready(1); light_lux(600);
        for(j=0;j<200 && !LightTask_RemoteLocked();++j) step(800,1);
        assert(LightTask_RemoteLocked());
        hx=light_task.hold_x; hy=light_task.hold_y; yaw=pidYaw.desired;
        channels[0]=1900; channels[1]=1100; channels[2]=1000;
        channels[3]=1900; channels[4]=1000;
        light_steps(200);
        assert(LightTask_RemoteLocked() && ALL_flag.height_lock && ALL_flag.flow_control);
        assert(pidYaw.desired==yaw && fabsf(pidHeightHigh.desired-800)<1);
        assert(pidPositionX.desired==hx && pidPositionY.desired==hy);
        assert(!AutoFlight_Active());
        channels[2]=1500; channels[3]=1500;
        light_steps(301); assert(light_task.state==LIGHT_COMPLETE);
        light_steps(20); assert(ALL_flag.height_lock && ALL_flag.flow_control);
    } else if (!strcmp(name,"light_manual_hold_emergency")) {
        manual_light_ready(1); light_lux(600);
        for(j=0;j<200 && !LightTask_RemoteLocked();++j) step(800,1);
        assert(LightTask_RemoteLocked());
        channels[2]=1000; channels[3]=1000; light_steps(4);
        assert(!ALL_flag.unlock && !LightTask_RemoteLocked() && TIM2->CCR1==0);
    } else if (!strcmp(name,"light_manual_hold_rc_loss")) {
        manual_light_ready(1); light_lux(600);
        for(j=0;j<200 && !LightTask_RemoteLocked();++j) step(800,1);
        assert(LightTask_RemoteLocked());
        for(j=0;j<510;++j) step(800,0);
        assert(!ALL_flag.unlock && !LightTask_RemoteLocked());
    } else if (!strcmp(name,"light_driver")) {
        uint32_t sample;
        assert(!bh1750.valid);
        for(j=0;j<38;++j) BH1750_Tick(0.006f);
        assert(mock_light_reads==0); /* 5 init commands + a full conversion. */
        BH1750_Tick(0.006f);
        assert(bh1750.valid && fabsf(bh1750.lux-300.0f)<0.01f);
        assert(mock_command_count==5);
        assert(!memcmp(mock_commands,(uint8_t[]){1,7,0x42,0x65,0x10},5));
        sample=bh1750.sample_id;
        mock_light_raw=0;
        for(j=0;j<34;++j) BH1750_Tick(0.006f);
        assert(bh1750.valid && bh1750.lux==0.0f && bh1750.sample_id==sample+1);
        mock_light_raw=65535U;
        for(j=0;j<34;++j) BH1750_Tick(0.006f);
        assert(!bh1750.valid && bh1750.saturated);
    } else if (!strcmp(name,"light_bus_failure")) {
        for(j=0;j<40;++j) BH1750_Tick(0.006f);
        assert(bh1750.valid);
        mock_light_bus_fails=1;
        for(j=0;j<34;++j) BH1750_Tick(0.006f);
        assert(!bh1750.valid && bh1750.errors);
        mock_light_bus_fails=0;
        for(j=0;j<220;++j) BH1750_Tick(0.006f);
        assert(bh1750.valid);
    } else if (!strcmp(name,"light_ground")) {
        light_lux(2000);
        for(j=0;j<500;++j) step(60,1);
        assert(light_task.state==LIGHT_WAIT_FLIGHT && !light_task.source_detected);
        assert(!LightTask_AlertActive() && !ALL_flag.unlock);
    } else if (!strcmp(name,"light_spike")) {
        light_ready();
        light_lux(450);
        for(j=0;j<34;++j) step(800,1);
        light_lux(300);
        light_steps(100);
        assert(light_task.state==LIGHT_WAIT_RISE && !light_task.source_detected);
    } else if (!strcmp(name,"light_gradual")) {
        light_ready();
        for(j=0;j<35;++j) { light_lux(300+j*5); light_steps(34); }
        assert(light_task.state==LIGHT_WAIT_RISE && !LightTask_AlertActive());
    } else if (!strcmp(name,"light_detection_alert")) {
        uint32_t start;
        light_detect(); start=alert_tick;
        while ((uint32_t)(SysTick_count-start)<1000U) {
            uint32_t ms=(uint32_t)(SysTick_count-start)*3U;
            PilotLED();
            assert(LightTask_AlertActive());
            assert(TIM3->CCR1==TIM3->CCR2 && TIM3->CCR2==TIM3->CCR3 && TIM3->CCR3==TIM3->CCR4);
            assert(TIM3->CCR1==(((ms/200U)&1U)?1000U:500U));
            step(800,1);
        }
        assert(!LightTask_AlertActive() && light_task.event_id==1);
        light_lux(300); light_steps(50); light_lux(450); light_steps(100);
        assert(light_task.event_id==1 && !LightTask_AlertActive());
    } else if (!strcmp(name,"light_manual_approach")) {
        light_detect();
        channels[0]=1900; channels[1]=1900; light_steps(20);
        assert(!LightTask_RemoteLocked() && AutoFlight_HorizontalInput());
        assert(pidPosRateX.desired==-15 && pidPosRateY.desired==15);
    } else if (!strcmp(name,"light_hold_lock")) {
        float yaw, hx, hy;
        light_enter_hold(); yaw=pidYaw.desired;
        hx=light_task.hold_x; hy=light_task.hold_y;
        channels[0]=1900; channels[1]=1100; channels[2]=1500;
        channels[3]=1900; channels[4]=2000;
        light_steps(200);
        assert(LightTask_RemoteLocked() && !AutoFlight_HorizontalInput());
        assert(auto_flight.state==AUTO_HOVER && !auto_flight.takeover_pending);
        assert(pidYaw.desired==yaw);
        assert(pidPositionX.desired==hx && pidPositionY.desired==hy);
        channels[2]=1000; light_steps(200);
        assert(LightTask_RemoteLocked() && auto_flight.state==AUTO_HOVER);
        light_steps(101);
        assert(light_task.state==LIGHT_COMPLETE && !LightTask_RemoteLocked());
        assert(light_task.stable_seconds==3.0f && AutoFlight_HorizontalInput());
        light_steps(4); /* Horizontal controller runs once every 24 ms. */
        assert(pidPosRateX.desired==-15 && pidPosRateY.desired==-15);
        assert(!auto_flight.takeover_pending); /* AUX1 during lock was discarded. */
        assert(auto_flight.state==AUTO_HOVER);
    } else if (!strcmp(name,"light_three_seconds")) {
        light_enter_hold();
        light_steps(499);
        assert(LightTask_RemoteLocked() && light_task.stable_seconds<3.0f);
        step(800,1);
        assert(light_task.state==LIGHT_COMPLETE);
        light_steps(700);
        assert(light_task.state==LIGHT_COMPLETE && light_task.event_id==1);
    } else if (!strcmp(name,"light_stability_reset")) {
        light_enter_hold(); light_steps(250);
        assert(light_task.stable_seconds>1.4f);
        Angle.roll=10.0f; light_steps(10);
        assert(light_task.stable_seconds==0.0f && LightTask_RemoteLocked());
        Angle.roll=0.0f;
        /* Attitude compensation also needs to settle before timing restarts. */
        for(j=0;j<500 && light_task.stable_seconds==0.0f;++j) step(800,1);
        assert(light_task.stable_seconds>0.0f);
        light_steps(498);
        assert(light_task.state==LIGHT_HOLD);
        step(800,1); assert(light_task.state==LIGHT_COMPLETE);
    } else if (!strcmp(name,"light_hold_emergency")) {
        light_enter_hold(); channels[3]=1000; step(800,1);
        assert(!ALL_flag.unlock && !LightTask_RemoteLocked() && TIM2->CCR1==0);
    } else if (!strcmp(name,"light_hold_rc_loss")) {
        light_enter_hold();
        for(j=0;j<510;++j) step(800,0);
        assert(!ALL_flag.unlock && !LightTask_RemoteLocked());
        assert(auto_flight.error==AUTO_ERROR_RC_LOSS);
    } else if (!strcmp(name,"light_hold_sensor_loss")) {
        light_enter_hold(); mock_light_bus_fails=1; light_steps(240);
        assert(light_task.state==LIGHT_ABORTED && !LightTask_RemoteLocked());
        assert(auto_flight.state==AUTO_HOVER && light_task.stable_seconds==0.0f);
    } else if (!strcmp(name,"light_hold_range_loss")) {
        light_enter_hold();
        for(j=0;j<30;++j) step(0,1);
        assert(auto_flight.state==AUTO_RANGE_FAULT);
        assert(light_task.state==LIGHT_ABORTED && !LightTask_RemoteLocked());
    } else if (!strcmp(name,"light_hold_dark")) {
        light_enter_hold(); light_lux(300); light_steps(240);
        assert(light_task.state==LIGHT_ABORTED && !LightTask_RemoteLocked());
    } else if (!strcmp(name,"light_bad_reconnect")) {
        light_ready(); mock_light_bus_fails=1; light_steps(50);
        mock_light_bus_fails=0; light_lux(1000); light_steps(500);
        assert(light_task.state==LIGHT_WAIT_RISE && !light_task.source_detected);
    } else if (!strcmp(name,"light_opt_in_reply")) {
        LightLinkStatus reply;
        light_detect();
        assert(mock_reply_count==0);
        mock_packet[20]='L'; mock_packet[21]='T'; mock_packet[22]=1; mock_packet[23]=0xA5;
        LightLink_OnControlFrame(mock_packet);
        assert(mock_reply_count==1 && LightLink_Decode(mock_reply,&reply));
        assert(reply.event_id==1 && (reply.flags & LIGHT_LINK_ALERT));
        assert(reply.ambient_lux==300 && reply.arrival_lux==550);
        assert(reply.diagnostic_valid && reply.auto_state==AUTO_HOVER);
        assert(reply.start_request_id==1 && reply.aux3==channels[6]);
        assert(abs((int)reply.height_mm-800)<=1 && (reply.sensor_flags & 7U)==7U);
        mock_packet[23]=0;
        LightLink_OnControlFrame(mock_packet); assert(mock_reply_count==1);
    } else if (!strcmp(name,"light_fault_led_priority")) {
        light_detect(); auto_flight.alarm=AUTO_ALARM_SENSOR;
        PilotLED();
        assert(TIM3->CCR1!=TIM3->CCR3); /* Fault pattern overrides all-light alert. */
    } else return 0;
    puts("passed");
    return 1;
}
