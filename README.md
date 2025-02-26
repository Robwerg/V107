# V107
**works on V1 PCB with ALS sensors (and OTT sensors with D9 data pin bridge)
**
ALS calibration is easier now, just edit the config file as per below:

ADC CONFIG__________________________________________________________

ADC0_Attached           =1;                                         //1 if in use. 0 if not in use.

ADC0_ID                 =AN;                                         //2 character ID key for selected sensor

ADC0_AveCount           =1;                                         //How many readings to average per measurement

ADC0_AveDelay           =10;                                        //Time between readings

ADC0_Cal_Temp           ={0};                                        //Leave empty for no temperature compensation

ADC0_Cal_mV_min         ={959};                      

ADC0_Cal_Val_min        ={0};                         

ADC0_Cal_mV_max         ={4903};

ADC0_Cal_Val_max        ={2000};


temperature compensation is addressed with hardware onboard the ALS sensors

**Still to come from Kai:**

Add config option to remove text identifiers from data so we don't have to remove in node-red

Add raw mV data on LoRa and SDcard

