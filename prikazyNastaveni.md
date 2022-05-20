    // rkArmSetGrabbing(false);
    // fmt::print("open: {}\n", rkArmGetServo(2) );
    // delay(1000);
    // rkArmSetGrabbing(true);
    // fmt::print("close: {}\n", rkArmGetServo(2) );
    
    // rkArmSetServo(3, 60);
    //rkArmSetServo(4, 60);

    // rkArmMoveTo(145, -45);   // je uprostred v prostoru
    // rkArmSetGrabbing(false); // open: 87 deg
    // delay(1000);
    // rkArmMoveTo(145, 65);   // je tesne nad zemi 
    // rkArmSetGrabbing(true); // close: 160 deg 
    
           //rkArmMoveTo(145, -45);   // je uprostred v prostoru
            //rkArmSetGrabbing(false); // open: 87 deg
            //delay(1000);
            //rkArmMoveTo(145, 65);   // je tesne nad zemi 
            //rkArmSetGrabbing(true); // close: 160 deg 

#ifdef AAA

rkArmSetServo(3, k);  // ID =3 -> leve klepeto dole cca 12 nahoře 80 smerem nahoru pozice roste 
rkArmSetServo(4, k);  // ID =4 -> prave klepeto dole 140 nahoře 60 smerem nahoru pozice klesa


#endif 

       

            // //rkMotorsSetPower(50, 50);
            // man.motor(MotorId::M1).drive(-500*110/100, 50);	// right motor 
            // man.motor(MotorId::M2).drive(-500, 50);	// left motor
            // delay(100);
            // //man.motor(MotorId::M1).drive(0, 0);	// nezastavi, dokud nedojede minuly drive  
            // //man.motor(MotorId::M2).drive(0, 0);
            // rkMotorsSetPower(0, 0); // zastavi ihned, i kdyz probiha drive  	
            // int32_t enR = man.motor(MotorId::M1).enc()->value();  // reading encoder
            // int32_t enL = man.motor(MotorId::M2).enc()->value();
            // fmt::print("{}: {},  {}\n", k, enL, enR);
              
            //rkArmSetServo(2, 120+k);  // 124 deg - rovnoběžná klepeta

    /*gpio_num_t SerialRx1 = GPIO_NUM_4;
    gpio_num_t SerialTx1 = GPIO_NUM_14;
    pinMode(SerialRx1, INPUT);
    pinMode(SerialTx1, OUTPUT);
    digitalWrite(SerialRx1, HIGH);
    digitalWrite(SerialTx1, HIGH);*/