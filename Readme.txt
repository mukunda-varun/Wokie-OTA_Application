----------COMPILER ENABLE/DISABLES----------
UART_DEBUG_EN					(line 306) -> Enabling UART debugging
OTA_EN		 					(line 308) -> Enables OTA Related variables & macros(Should change flash.ld file for flash address)

WOKIE_GREEN_BRD_MC_EN			(line 311) -> Used for enabling the compiler macros required for Green Board(Wokie)
WOKIE_GREEN_V_1_5_EN			(line 313) -> Used for enabling the compiler macros required for Latest Green Brd V1.5 (Wokie)
ACT_MACHINE_EN					(line 315) -> Used for enabling the compiler macros required for Black Board(ACT)
WOKIE_BLACK_BRD_EN				(line 317) -> Used for enabling the compiler macros required for Black Board(Wokie)

TM1668_DRIVE_ENABLE				(line 321) -> Enables the functions related to fetch & execute of Green board induction(Either TM1668/BS84C12A can be used at a time currently)
BS84C12A_DRIVE_ENABLE			(line 323) -> Enables the functions related to fetch & execute of Black board induction(Either TM1668/BS84C12A can be used at a time currently)
ANDROID_INTERFACE_ENABLE		(line 325) -> Enables the functions Android apk interface
IR_TEMPERATURE_SENSOR_ENABLE	(line 327) -> Enables the Reading of IR temperature sensor
SPEED_SENSOR_ENABLE				(line 329) -> Enables the Speed sensor related function calls for getting speed
DC_MOTOR_TASK_ENABLE			(line 331) -> Enables the DC Motor Task(Required for wokie only)
SPARE_DC_PWM_EN					(line 333) -> Enables the Spare DC Motor timer & running of spare motor(Either Spare/Drum can be used at a time currently).
DRUM_DC_PWM_EN					(line 335) -> Enables the Drum DC Motor timer & running of Drum motor(Either Spare/Drum can be used at a time currently).
WATTAGE_MODE_ENABLE				(line 337) -> Enables the functions related execution of Wattage mode
TEMPERATURE_CONTROL_MODE		(line 339) -> Enables the functions related executing Temperature control mode
TEMPERATURE_CURVE  				(line 341) -> Enables/Disables the temperature curve mode
DELTA_METHOD					(line 343) -> Enable/Disable the temperature control mode with delta mode/hard coded temperature curve(with ranges)
GET_DISPLAY_WITH_ADC			(line 345) -> Enables the data fetch of 7 segment with BS84C12A using ADC method(not used)
PROCESS_ERROR_ENABLE			(line 347) -> Enables Error mechanisms of the machine(wokie)
ANDROID_MC_SETTINGS				(line 349) -> Enables the android machine settings
ANDROID_ACK_CHECK_EN			(line 351) -> Enables the android acknowledge check for avoiding error condition stop during crash of APK

LED_BUZZER_OTA					(line 455) -> GPIO Toggling during bootup selection, mainly for OTA(0 = DISCOVERY LED, 1 = PCB BUZZER)

For OTA Enabling need to update the macro in line number 301 to 1 andd change few things in flash.ld file.
----------COMPILER ENABLE/DISABLES----------