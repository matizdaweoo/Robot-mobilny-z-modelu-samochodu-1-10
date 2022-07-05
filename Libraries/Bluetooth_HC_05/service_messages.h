//Automotive lighting (oswietlenie samochodowe)
#define AutomotiveLighting		'L'
	#define DaytimeRunningLamps		'0'			//Swiatla do jazdy dziennej
	#define FrontPositionLampss		'1'			//Swiatla pozycyjne przednie
	#define DippedBeam				'2'			//Swiatla krotkie
	#define MainBeam				'3'			//Swiatla dlugie
	#define FrontFogLamps			'4'			//Swiatla przeciwmglowe przednie
	#define RearFogLamps			'5'			//Swiatla przeciwmglowe tylnie
	#define HazardFlashers			'6'			//Awaryjne
	#define LeftTurnSignal			'7'			//Lewy migacz
	#define RightTurnSignal			'8'			//Prawy migacz
	#define Horn					'H'			//Klakson

//Control mobile robot (sterowanie robota mobilnego)
#define RobotControl			'C'
	#define AutoControl				'A'
	#define	ManualControl			'M'

	#define ForwardDirection		'1'			//Jazda do przodu
	#define ReverseDirection		'2'			//Jazda do tylu
	#define TurnLeftWheels			'3'			//Skret w lewo
	#define TurnRightWheels			'4'			//Skret w prawo
	#define FDTLW					'5'			//Jazda do przodu w lewo
	#define FDTRW					'6'			//Jazda do przodu w prawo
	#define RDTLW					'7'			//Jazda do tylu w lewo
	#define RDTRW					'8'			//Jazda do tylu w prawo
	#define Break					'9'			//Hamulec
	#define Neutral					'0'			//Wylaczenie napedu

//Gear stick (drazek zmiany biegow)
#define Gear					'G'
	#define GearAutomatic			'A'
	#define GearManual				'M'
	#define GearN					'N'		//Bieg neutralny
	#define Gear1					'1'		//Bieg pierwszy
	#define Gear2					'2'		//Bieg drugi
	#define Gear3					'3'		//Bieg trzeci
	#define Gear4					'4'		//Bieg czwarty
	#define Gear5					'5'		//Bieg piaty
	#define Gear6					'6'		//Bieg szosty
	#define GearR					'R'		//Bieg Wsteczny

//Other instructions (inne instrukcje)
#define Meansure				'M'
#define VoltageBattery			'1'		//Napiecie akumulatora
#define CurrentBattery			'2'		//Prad akumulatora

void ObslugaKomunikatow(char tabInstr[], int Rozmiar);
void ResetStopReversLamps(void);
void ResetStopSetReversLamps(void);
