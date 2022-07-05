using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Globalization;
using System.Data;
using System.Drawing;
using System.Drawing.Drawing2D;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;
using System.IO.Ports;
using System.Windows.Input;

namespace Aplication_for_mobile_robot
{
    public partial class Form1 : Form
    {
        /*------------------------------ Deklaracje / definicje nowego obiektu ------------------------------*/
        CultureInfo US_Culturenfo = new CultureInfo("en-US");   //Tworze nowy format zapisu liczb po przecinku z PL (0,43) na US(0.43) [w takim formacie beda wysylanie wsp do robota (04.54)]

        SerialPort sp = new SerialPort();   //Deklaracja
        
        Graphics GraphicRysunekRadaru;
        Graphics GraphicRysowanieOsiWspolrzednych;
        Bitmap bitmapaRysowanieOsiWspolrzednych;
        Graphics GraphicOknoZachowanieAlgorytmu;
        Bitmap bitmapaOknoZachowanieAlgorytmu;
        
        Graphics rysunekAutomatuStanu;

        FormSendDataToMobileRobot formSendData = new FormSendDataToMobileRobot();
        /*------------------------------ Ustawienia domyslne portu szeregowego ------------------------------*/
        string UART_PortName = "COM9";                                                     
        int UART_BaudRate = 9600, UART_DataBits = 8, UART_Parity = 0, UART_StopBit = 1;
        string readDataBuffor;
        string newReadDataBuffor, oldReadDataBuffor;

        char ActiveGear = 'N';
        float SpeedMobileRobot = 30;
        float[] tabSpeedMobileRobot = new float[20] { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 };
        float TempCPUMobileRobot = 20;

        //Kontrola z klawiatury
        bool keyboardIsActive = false;

        /*------------------------------ Flagi okreslajace ktore komendy wymagaja potwierdzenia ------------------------------*/
        bool sendMSFMR_flag = false;    //Potwierdzenie ograniczenia predkosci robota
        bool sendDSA1_flag = false, sendDSA2_flag = false, sendDSA3_flag = false, sendDSA4_flag = false, sendDSA5_flag = false;
        
        bool sendGSA_flag = false, sendGSW_flag = false, sendGSG_flag = false;

        /*------------------------------ Flagi okreslajace ktore komendy wymagaja potwierdzenia ------------------------------*/
        bool sendPSX_flag = false, sendPSY_flag = false, sendPSO_flag = false;  //Position Start
        bool sendPGX_flag = false, sendPGY_flag = false, sendPGO_flag = false;  //Position Goal
        bool sendPWA_flag = false, sendPAS_flag = false;
        string xStartWsp = null, yStartWsp = null, fiStartWsp = null;
        string xStopWsp = null, yStopWsp = null, fiStopWsp = null;
        string checkAlgorytm = null;
        int progressBarSendDataToMobileRobot = 0;
        
        /*------------------------------ Flagi okreslajace ktore komendy wymagaja potwierdzenia ------------------------------*/
        //Rodzaj sterowania
        //bool sendCM_flag = false, sendCA_flag = false;
        //Rodzaj skrzyni biegow
        bool sendGM_flag = false, sendGA_flag = false;
        //Bieg
        bool sendN_flag = false, send1_flag = false, send2_flag = false, send3_flag = false, send4_flag = false, send5_flag = false, send6_flag = false, sendR_flag = false;
        //Oswietlenie
        bool sendL0_flag = false, sendL1_flag = false, sendL2_flag = false, sendL3_flag = false, sendL5_flag = false, sendL6_flag = false, sendL7_flag = false, sendL8_flag = false, sendL9_flag = false;
        //bool sendL4_flag = false;
        /*------------------------------ Zmienne sluzace do okreslena aktualnego stanu oswietlenia robota mobilnego ------------------------------*/
        bool StanSwiatlaDzienne = false, StanSwiatlaPostojowe = false, StanSwiatlaKrotkie = false, StanSwiatlaDlugie = false;
        bool StanSwiatlaAwaryjne = false, StanMigaczLewy = false, StanMigaczPrawy = false, StanKlakson = false;
        bool StanSwiatlaPrzeciwmgloweTylne = false; //, StanSwiatlaPrzeciwmglowePrzednie = false; 
        int EnableTimerMigacze = 0; bool StanTimerMigacze = false;

        /*------------------------------ Zmienne uzywane w funkcji do wysowania siatki zasiegu radaru ------------------------------*/
        int x_srodekTyl = 138, y_srodek = 137; int x_srodekPrzod = 265; int odstepSiatki = 20; int srednicaOkregu = 82; int promienOkregu = 41;

        int wysokoscStrzalki = 15;
        int wysokoscOkna = 0, szerokoscOkna = 0;
        int offsetKratkix = 0, offsetKratkiy = 0;
        double numx = 0.0, numy = 0.0;
        /*------------------------------ Tablica przechowujaca dane z czujnikow odleglosci ------------------------------*/
        int[] DaneOdleglosciPrzod = new int[7] {0, 0, 0, 0, 0, 0, 0};    //Pierwsza liczba mowi o czujnikach z przodu (0) lub z tylu (1), ostatnia reprezentuje czujnik ultradzwiekowy
        int[] DaneOdleglosciTyl = new int[7] {1, 0, 0, 0, 0, 0, 0};

        int maxSpeedForMobileRobot = 30; //Zakres od 0,1[m/s] do 2,5 [m/s] 
        string stringMaxSFMR = null;

        char stanAutomatuWanderGoal = '0';
        int distSensActive1 = 15, distSensActive2 = 30, distSensActive3 = 30, distSensActive4 = 30, distSensActive5 = 15; //Zakres od 10 do 80
        string stringDSA1 = null, stringDSA2 = null, stringDSA3 = null, stringDSA4 = null, stringDSA5 = null;

        char rodzajZachowaniaMS = '0';
        int[] vectorMagnitude = new int[8] {0, 0, 0,  0,   0,   0,   0,   0 };
        int[] vectorDirection = new int[8] {0, 0, 0, 90, 135, 180, 225, 270 };
        double gainAvoid = 1.0;    //Zakres od 0,0 do 50,0
        double gainWander = 1.0, gainGoal = 1.0;
        string stringGSA = null;
        string stringGW = null, stringGG = null;

        //int angle = 0;

        public double xStart = 0.0, yStart = 0.0, xStop = 2.0, yStop = 1.0;    //Wspolrzedne punktu poczatkowego i koncowego do ktorych ma dojechac robot mobilny
        public int fiStart = 0, fiStop = 40;

        double xActual = 0.0, yActual = 0.0;
        int fiActual = 0;
        double[] tabXActual = new double[1] { 0.0 };
        double[] tabYActual = new double[1] { 0.0 };
        int nrWspXYActual = 0;
        double xMinValue = 0.0, xMaxValue = 0.0;    //Zmienne potrzebne do okreslenia skrajnych wartosci
        double yMinValue = 0.0, yMaxValue = 0.0;

        public int szerokoscKratkix = 0, szerokoscKratkiy = 0;
        public char wybranyAlgorytm = '0';
        public bool wyslanoDanePktDoRobota = false;

        bool algorytmStart = false;

        bool sendNPO_flag = false;
        string PoleObiektu = null;
        int nowePoleObiektu = 0;

        public Form1()
        {
            InitializeComponent();
            GetAvailablePorts();

            /*      Zakłądka "Sterowanie reczne"     */
            
            /*      Zakłądka "Sterowanie autonomiczne"     */
            textBox_xPktStart.Text       = Convert.ToString(xStart);
            textBox_yPktStart.Text       = Convert.ToString(yStart);
            textBox_fiPktStart.Text      = Convert.ToString(fiStart);
            textBox_xPktStop.Text        = Convert.ToString(xStop);
            textBox_yPktStop.Text        = Convert.ToString(yStop);
            textBox_fiPktStop.Text       = Convert.ToString(fiStop);

            textBox_AlgSubMaxSpeed.Text  = Convert.ToString(maxSpeedForMobileRobot);
            textBox_DistSensActive1.Text = Convert.ToString(distSensActive1);
            textBox_DistSensActive2.Text = Convert.ToString(distSensActive2);
            textBox_DistSensActive3.Text = Convert.ToString(distSensActive3);
            textBox_DistSensActive4.Text = Convert.ToString(distSensActive4);
            textBox_DistSensActive5.Text = Convert.ToString(distSensActive5);

            textBox_AlgMotMaxSpeed.Text  = Convert.ToString(maxSpeedForMobileRobot);
            textBox_GainAvoid.Text       = Convert.ToString(gainAvoid);
            textBox_GainWander.Text      = Convert.ToString(gainWander);
            textBox_GainGoal.Text        = Convert.ToString(gainGoal);

            /*      Zakłądka "Ustawienia"     */
            comboBox_PortName.Text = UART_PortName;
            comboBox_BaudRate.Text = Convert.ToString(UART_BaudRate);
            comboBox_DataBits.Text = Convert.ToString(UART_DataBits);
            comboBox_Parity.Text = Convert.ToString(UART_Parity);
            comboBox_StopBits.Text = Convert.ToString(UART_StopBit);
        }

        private void tabControl1_Click(object sender, EventArgs e)
        {
            RysowanieSiatkiRadaru();
            RysowanieOsiPredkosci();
            RysowanieOsiWspolrzednych(xStart, yStart, xStop, yStop);

            if(wybranyAlgorytm == '2')
            {
                RysowanieAutomatuStanu(stanAutomatuWanderGoal);
            }
            if(wybranyAlgorytm == '3')
            {
                RysowanieWektorowStrowan(vectorMagnitude, vectorDirection, rodzajZachowaniaMS);
                //groupBox_ZachowanieAlgorytmu.Text = "Algorytm Motor Schemas";
            }

            if (wyslanoDanePktDoRobota == true)
            {
                if (wybranyAlgorytm == '1')
                {
                    if (wybranyAlgorytm == '1')
                    {
                        RysowanieSciezkiRobota(tabXActual, tabYActual);
                        RysowanieUkladuWspolrzednych(xActual, yActual, fiActual, szerokoscKratkix, szerokoscKratkiy, "orange");
                    }

                    RysowanieUkladuWspolrzednych(xStop, yStop, fiStop, szerokoscKratkix, szerokoscKratkiy, "red");
                }
                RysowanieUkladuWspolrzednych(xStart, yStart, fiStart, szerokoscKratkix, szerokoscKratkiy, "green");
            }
        }

        private void Form1_Resize(object sender, EventArgs e)
        {
            RysowanieSiatkiRadaru();
            RysowanieOsiPredkosci();
            
            if (wybranyAlgorytm == '1')
            {
                RysowanieOsiWspolrzednych(xStart, yStart, xStop, yStop);
                RysowanieSciezkiRobota(tabXActual, tabYActual);
                RysowanieUkladuWspolrzednych(xActual, yActual, fiActual, szerokoscKratkix, szerokoscKratkiy, "orange");
                RysowanieUkladuWspolrzednych(xStop, yStop, fiStop, szerokoscKratkix, szerokoscKratkiy, "red");
                RysowanieUkladuWspolrzednych(xStart, yStart, fiStart, szerokoscKratkix, szerokoscKratkiy, "green");
            }

            else if (wybranyAlgorytm == '2')
            {
                RysowanieAutomatuStanu(stanAutomatuWanderGoal);

                RysowanieOsiWspolrzednych(xMinValue, yMinValue, xMaxValue, yMaxValue);
                RysowanieSciezkiRobota(tabXActual, tabYActual);
                RysowanieUkladuWspolrzednych(xActual, yActual, fiActual, szerokoscKratkix, szerokoscKratkiy, "orange");
                RysowanieUkladuWspolrzednych(xStart, yStart, fiStart, szerokoscKratkix, szerokoscKratkiy, "green");
            }
            else if (wybranyAlgorytm == '3')
            {
                RysowanieWektorowStrowan(vectorMagnitude, vectorDirection, rodzajZachowaniaMS);

                RysowanieOsiWspolrzednych(xMinValue, yMinValue, xMaxValue, yMaxValue);
                RysowanieSciezkiRobota(tabXActual, tabYActual);
                RysowanieUkladuWspolrzednych(xActual, yActual, fiActual, szerokoscKratkix, szerokoscKratkiy, "orange");
                RysowanieUkladuWspolrzednych(xStart, yStart, fiStart, szerokoscKratkix, szerokoscKratkiy, "green");
            }
            else
            {
                RysowanieOsiWspolrzednych(xStart, yStart, xStop, yStop);
            }
        }
        
        public void form2ButtonOK()
        {
            sendDataToMobileRobot("PSD", 'n');

            WyczyscWykresPredkosci();
            WyczyscSiatkeRadaru();

            RysowanieOsiWspolrzednych(xStart, yStart, xStop, yStop);

            RysowanieUkladuWspolrzednych(xStart, yStart, fiStart, szerokoscKratkix, szerokoscKratkiy, "green");
            if (wybranyAlgorytm == '1')
            {
                RysowanieUkladuWspolrzednych(xStop, yStop, fiStop, szerokoscKratkix, szerokoscKratkiy, "red");
                groupBox_ZachowanieAlgorytmu.Text = "";
            }
            if (wybranyAlgorytm == '2')
            {
                groupBox_ZachowanieAlgorytmu.Text = "Algorytm Subsumption";
                stanAutomatuWanderGoal = '0';
                RysowanieAutomatuStanu(stanAutomatuWanderGoal);
            }
            if (wybranyAlgorytm == '3')
            {
                groupBox_ZachowanieAlgorytmu.Text = "Algorytm Motor Schemas";
                WyczyscWektorySterowan();
                RysowanieWektorowStrowan(vectorMagnitude, vectorDirection, rodzajZachowaniaMS);
            }
            wyslanoDanePktDoRobota = true;
            Button_StartAlgorytm.Enabled = true;
            Button_StartAlgorytm.Focus();
        }
        public void form2ButtonAnuluj()
        {
            Button_Algorytm1.BackColor = Color.Transparent;
            Button_Algorytm2.BackColor = Color.Transparent;
            Button_Algorytm3.BackColor = Color.Transparent;

            wyslanoDanePktDoRobota = false;

            sendPWA_flag = false;
            wybranyAlgorytm = '0';

            groupBox_ZachowanieAlgorytmu.Text = "";

            sendPSX_flag = false; sendPSY_flag = false; sendPSO_flag = false;
            ClearTextBoxsPktStart();
            sendPGX_flag = false; sendPGY_flag = false; sendPGO_flag = false;
            ClearTextBoxsPktStop();

            //zerowanie flag algorytmu 2
            sendMSFMR_flag = false;
            sendDSA1_flag = false; sendDSA2_flag = false; sendDSA3_flag = false; sendDSA4_flag = false; sendDSA5_flag = false;
            //zerowanie flag algorytmu 3
            sendGSA_flag = false;  sendGSW_flag = false; sendGSG_flag = false;

            RysowanieOsiWspolrzednych(xStart, yStart, xStop, yStop);

            Button_WyslijDaneDoRobota.Enabled = false;
            textBox_xPktStart.Enabled = false;
            textBox_yPktStart.Enabled = false;
            textBox_fiPktStart.Enabled = false;
            textBox_xPktStop.Enabled = false;
            textBox_yPktStop.Enabled = false;
            textBox_fiPktStop.Enabled = false;
            Button_StartAlgorytm.Enabled = false;
            Button_StopAlgorytm.Enabled = false;
        }

        public void RysowanieOsiWspolrzednych(double xStart, double yStart, double xStop, double yStop)
        {
            try
            {
                bitmapaRysowanieOsiWspolrzednych = new Bitmap(pictureBox_WykresPolozeniaRobota.Size.Width, pictureBox_WykresPolozeniaRobota.Size.Height);
                GraphicRysowanieOsiWspolrzednych = Graphics.FromImage(bitmapaRysowanieOsiWspolrzednych);
                GraphicRysowanieOsiWspolrzednych.Clear(Color.White);

                wysokoscOkna = (pictureBox_WykresPolozeniaRobota.Size.Height - 1);
                szerokoscOkna = (pictureBox_WykresPolozeniaRobota.Size.Width - 1);
                double ogrxStart = 0.0, ogryStart = 0.0, ogrxStop = 0.0, ogryStop = 0.0;
                double ogrxStartRound = 0.0, ogryStartRound = 0.0, ogrxStopRound = 0.0, ogryStopRound = 0.0;
                int szerokoscOsix = 0, szerokoscOsiy = 0;
                //int iloscKratekx = 0, iloscKrateky = 0;
                int noweWspOsix = 0, noweWspOsiy = 0;
                int offsetOsix = 0, offsetOsiy = 0;
                int offsetx = 0, offsety = 0;

                Font drawFont1 = new Font("Arial", 8);
                Font drawFont2 = new Font("Arial", 10);
                SolidBrush drawBrush = new SolidBrush(Color.Black);
                SolidBrush drawBrushGreen = new SolidBrush(Color.Green);
                SolidBrush drawBrushRed = new SolidBrush(Color.Red);
                Pen blackPen = new Pen(Color.Black, 1);
                Pen greenPen = new Pen(Color.LimeGreen, 1);
                Pen redPen = new Pen(Color.Red, 1);
                Pen lightgrayPen = new Pen(Color.LightGray, 1);
                Rectangle kolo = new Rectangle(x_srodekTyl - promienOkregu, y_srodek - promienOkregu, 4, 4);
                Rectangle ramkaOsiWsp = new Rectangle(0, 0, szerokoscOkna, wysokoscOkna);

                xStart = xStart * 10; yStart = yStart * 10; xStop = xStop * 10; yStop = yStop * 10;     //Zmiana jednostki z [m] na [dm] zeby moc rysowac pkt co 0.1 [m]

                if (xStop == 0.0 && xStart == 0.0 && yStop == 0.0 && yStart == 0.0)    //Wykres pokazowy rysowany na starcie lub gdy nie wpisano danych pkt start i koniec
                {
                    ogrxStart = xStart - 20;
                    ogryStart = yStart - 20;
                    ogrxStop = xStop + 20;
                    ogryStop = yStop + 20;
                    ogrxStartRound = Math.Round((ogrxStart / 10), 0) * 10;              //Zaokraglam wsp wejsciowe zeby rozszerzyc zakres gdy wpiszemy np. 3,5
                    ogryStartRound = Math.Round((ogryStart / 10), 0) * 10;
                    ogrxStopRound = Math.Round((ogrxStop / 10), 0) * 10;
                    ogryStopRound = Math.Round((ogryStop / 10), 0) * 10;
                    numx = ogrxStartRound;  //
                    numy = ogryStartRound;
                }
                else if ((xStop > xStart) && (yStop >= yStart))        //I cwiartka jazda z dolu do gory
                {
                    if (xStart > 0) ogrxStart = -10; else ogrxStart = xStart - 10;      //Zakres skali x z lewej strony tak aby zawsze uwzględnić oś y
                    if (yStart > 0) ogryStart = -10; else ogryStart = yStart - 10;      //Zakres skali y od dolu strony tak aby zawsze uwzględnić oś y
                    if (xStop < 0) ogrxStop = 10; else ogrxStop = xStop + 10;           //Zakres skali x z prawej strony
                    if (yStop < 0) ogryStop = 10; else ogryStop = yStop + 10;           //Zakres skali y od gory
                    ogrxStartRound = Math.Round((ogrxStart / 10), 0) * 10;              //Zaokraglam wsp wejsciowe zeby rozszerzyc zakres gdy wpiszemy np. 3,5
                    ogryStartRound = Math.Round((ogryStart / 10), 0) * 10;
                    ogrxStopRound = Math.Round((ogrxStop / 10), 0) * 10;
                    ogryStopRound = Math.Round((ogryStop / 10), 0) * 10;
                    numx = ogrxStartRound;
                    numy = ogryStartRound;
                }
                else if ((xStop <= xStart) && (yStop > yStart))  //II cwiartka jazda z dolu do gory
                {
                    if (xStart < 0) ogrxStart = 10; else ogrxStart = xStart + 10;       //Zakres skali x z lewej strony tak aby zawsze uwzględnić oś y
                    if (yStart > 0) ogryStart = -10; else ogryStart = yStart - 10;      //Zakres skali y od dolu strony tak aby zawsze uwzględnić oś y
                    if (xStop > 0) ogrxStop = -10; else ogrxStop = xStop - 10;          //Zakres skali x z prawej strony
                    if (yStop < 0) ogryStop = 10; else ogryStop = yStop + 10;           //Zakres skali y od gory
                    ogrxStartRound = Math.Round((ogrxStart / 10), 0) * 10;              //Zaokraglam wsp wejsciowe zeby rozszerzyc zakres gdy wpiszemy np. 3,5
                    ogryStartRound = Math.Round((ogryStart / 10), 0) * 10;
                    ogrxStopRound = Math.Round((ogrxStop / 10), 0) * 10;
                    ogryStopRound = Math.Round((ogryStop / 10), 0) * 10;
                    numx = ogrxStopRound;
                    numy = ogryStartRound;
                }
                else if ((xStop < xStart) && (yStop <= yStart))    //III cwiartka jazda z gory do dolu
                {
                    if (xStart < 0) ogrxStart = 10; else ogrxStart = xStart + 10;       //Zakres skali x z lewej strony tak aby zawsze uwzględnić oś y
                    if (yStart < 0) ogryStart = 10; else ogryStart = yStart + 10;       //Zakres skali y od dolu strony tak aby zawsze uwzględnić oś y
                    if (xStop > 0) ogrxStop = -10; else ogrxStop = xStop - 10;          //Zakres skali x z prawej strony
                    if (yStop > 0) ogryStop = -10; else ogryStop = yStop - 10;          //Zakres skali y od gory
                    ogrxStartRound = Math.Round((ogrxStart / 10), 0) * 10;              //Zaokraglam wsp wejsciowe zeby rozszerzyc zakres gdy wpiszemy np. 3,5
                    ogryStartRound = Math.Round((ogryStart / 10), 0) * 10;
                    ogrxStopRound = Math.Round((ogrxStop / 10), 0) * 10;
                    ogryStopRound = Math.Round((ogryStop / 10), 0) * 10;
                    numx = ogrxStopRound;
                    numy = ogryStopRound;
                }
                else if ((xStop >= xStart) && (yStop < yStart))    //IV cwiartka jazda z gory do dolu
                {
                    if (xStart > 0) ogrxStart = -10; else ogrxStart = xStart - 10;      //Zakres skali x z lewej strony tak aby zawsze uwzględnić oś y
                    if (yStart < 0) ogryStart = 10; else ogryStart = yStart + 10;       //Zakres skali y od dolu strony tak aby zawsze uwzględnić oś y
                    if (xStop < 0) ogrxStop = 10; else ogrxStop = xStop + 10;           //Zakres skali x z prawej strony
                    if (yStop > 0) ogryStop = -10; else ogryStop = yStop - 10;          //Zakres skali y od gory
                    ogrxStartRound = Math.Round((ogrxStart / 10), 0) * 10;              //Zaokraglam wsp wejsciowe zeby rozszerzyc zakres gdy wpiszemy np. 3,5
                    ogryStartRound = Math.Round((ogryStart / 10), 0) * 10;
                    ogrxStopRound = Math.Round((ogrxStop / 10), 0) * 10;
                    ogryStopRound = Math.Round((ogryStop / 10), 0) * 10;
                    numx = ogrxStartRound;
                    numy = ogryStopRound;
                }

                if (xStop == 0.0 && yStop == 0.0 && (wybranyAlgorytm == '2' || wybranyAlgorytm == '3'))    //Wykres pokazowy rysowany na starcie lub gdy nie wpisano danych pkt start i koniec
                {
                    ogrxStart = xStart - 20;
                    ogryStart = yStart - 20;
                    ogrxStop = xStart + 20;
                    ogryStop = yStart + 20;
                    ogrxStartRound = Math.Round((ogrxStart / 10), 0) * 10;              //Zaokraglam wsp wejsciowe zeby rozszerzyc zakres gdy wpiszemy np. 3,5
                    ogryStartRound = Math.Round((ogryStart / 10), 0) * 10;
                    ogrxStopRound = Math.Round((ogrxStop / 10), 0) * 10;
                    ogryStopRound = Math.Round((ogryStop / 10), 0) * 10;
                    numx = ogrxStartRound;
                    numy = ogryStartRound;
                }

                if (ogrxStartRound <= ogrxStopRound)            //Wybieram wsp x ktora jest najbardziej z lewej strony. Potrzebna jest do okreslania offsetu wykresu
                { offsetx = Convert.ToInt32(ogrxStartRound); }
                else
                { offsetx = Convert.ToInt32(ogrxStopRound); }

                if (ogryStartRound <= ogryStopRound)            //Wybieram wsp y ktora jest najnizej. Potrzebna jest do okreslania offsetu wykresu
                    offsety = Convert.ToInt32(ogryStartRound);
                else
                    offsety = Convert.ToInt32(ogryStopRound);

                szerokoscOsix = Convert.ToInt32(Math.Abs(ogrxStopRound - ogrxStartRound));   //Obliczenie zakresy na osi x dla zaokraglonego zakresu
                szerokoscOsiy = Convert.ToInt32(Math.Abs(ogryStopRound - ogryStartRound));   //Obliczenie zakresy na osi y dla zaokraglonego zakresu

                if (szerokoscOsix == 0)         //Jesli xStart == xStop
                {
                    szerokoscKratkiy = (wysokoscOkna - (wysokoscStrzalki * 2)) / szerokoscOsiy;                             //
                    offsetKratkiy = ((wysokoscOkna - (wysokoscStrzalki * 2)) - (szerokoscKratkiy * szerokoscOsiy)) / 2;
                    szerokoscKratkix = szerokoscKratkiy;
                    offsetKratkix = ((szerokoscOkna - (wysokoscStrzalki * 2)) - (szerokoscKratkix * szerokoscOsix)) / 2;    //Obliczenie offsetu w przypadku wyniki kratki z czescia ulamkowa (np. 54,3452 kratki -> 54 (dopuszczalne)
                }
                else if (szerokoscOsiy == 0)    //Jesli yStart == yStop
                {
                    szerokoscKratkix = (szerokoscOkna - (wysokoscStrzalki * 2)) / szerokoscOsix;                            //Obliczenie szerokosci kratki (podzialki) w zaleznosci od szerokosci okna i ilosci podzialek (brany pod uwage wynik calkowity!)
                    offsetKratkix = ((szerokoscOkna - (wysokoscStrzalki * 2)) - (szerokoscKratkix * szerokoscOsix)) / 2;    //Obliczenie offsetu w przypadku wyniki kratki z czescia ulamkowa (np. 54,3452 kratki -> 54 (dopuszczalne)
                    szerokoscKratkiy = szerokoscKratkix;                           //
                    offsetKratkiy = ((wysokoscOkna - (wysokoscStrzalki * 2)) - (szerokoscKratkiy * szerokoscOsiy)) / 2;
                }
                else
                {
                    szerokoscKratkix = (szerokoscOkna - (wysokoscStrzalki * 2)) / szerokoscOsix;                            //Obliczenie szerokosci kratki (podzialki) w zaleznosci od szerokosci okna i ilosci podzialek (brany pod uwage wynik calkowity!)
                    offsetKratkix = ((szerokoscOkna - (wysokoscStrzalki * 2)) - (szerokoscKratkix * szerokoscOsix)) / 2;    //Obliczenie offsetu w przypadku wyniki kratki z czescia ulamkowa (np. 54,3452 kratki -> 54 (dopuszczalne)
                    szerokoscKratkiy = (wysokoscOkna - (wysokoscStrzalki * 2)) / szerokoscOsiy;                             //
                    offsetKratkiy = ((wysokoscOkna - (wysokoscStrzalki * 2)) - (szerokoscKratkiy * szerokoscOsiy)) / 2;
                }

                offsetOsix = offsety - 0;               //Obliczma czy na osi y sa wartosci ujemne
                if (offsetOsix < 0)                     //Jesli tak to poszerz zakres w d
                    offsetOsix = Math.Abs(offsety);     // + 10;
                else                                    //Tu jesli wartosc y jest >= 0
                    offsetOsix = 10;

                offsetOsiy = offsetx - 0;
                if (offsetOsiy < 0)
                    offsetOsiy = Math.Abs(offsetx);     // + 10;
                else
                    offsetOsiy = 10;

                //Wyznaczenie wsp. osi x i y
                noweWspOsix = wysokoscOkna - (wysokoscStrzalki + (szerokoscKratkiy * offsetOsix) + offsetKratkiy);
                noweWspOsiy = wysokoscStrzalki + (szerokoscKratkix * offsetOsiy) + offsetKratkix;
                //Rysowanie osi wsp.
                GraphicRysowanieOsiWspolrzednych.DrawLine(blackPen, 0, noweWspOsix, szerokoscOkna, noweWspOsix);                             //Os odcietych (x)
                GraphicRysowanieOsiWspolrzednych.DrawLine(blackPen, szerokoscOkna, noweWspOsix, (szerokoscOkna - 8), (noweWspOsix - 3));     //Strzalka dla osi x
                GraphicRysowanieOsiWspolrzednych.DrawLine(blackPen, szerokoscOkna, noweWspOsix, (szerokoscOkna - 8), (noweWspOsix + 3));
                GraphicRysowanieOsiWspolrzednych.DrawString("x", drawFont2, drawBrush, (szerokoscOkna - 9), (noweWspOsix + 2));              //Podpis osi
                GraphicRysowanieOsiWspolrzednych.DrawLine(blackPen, noweWspOsiy, 0, noweWspOsiy, wysokoscOkna);                              //Os rzednych (y)
                GraphicRysowanieOsiWspolrzednych.DrawLine(blackPen, noweWspOsiy, 0, (noweWspOsiy - 3), 8);                                   //Strzalka dla osi y
                GraphicRysowanieOsiWspolrzednych.DrawLine(blackPen, noweWspOsiy, 0, (noweWspOsiy + 3), 8);
                GraphicRysowanieOsiWspolrzednych.DrawString("y", drawFont2, drawBrush, (noweWspOsiy + 5), -6);                               //Podpis osi

                //Rysowanie podziałki i oznaczeń dla osi x
                for (int i = 0; i < (szerokoscOsix + 1); i++)           //Mala podzialka osi x (co 0.1)
                {
                    int xDrawLine = wysokoscStrzalki + offsetKratkix + (i * szerokoscKratkix);
                    GraphicRysowanieOsiWspolrzednych.DrawLine(blackPen, xDrawLine, (noweWspOsix - 2), xDrawLine, (noweWspOsix + 2));
                }
                for (int i = 0; i < ((szerokoscOsix / 10) + 1); i++)    //Duza podzialka osi x (co 1.0)
                {
                    int xDrawLine = Convert.ToInt32(wysokoscStrzalki + offsetKratkix + (i * (szerokoscKratkix * 10)));
                    int a = Convert.ToInt32((numx / 10) + i);

                    if (a < 0)
                    {
                        GraphicRysowanieOsiWspolrzednych.DrawLine(lightgrayPen, xDrawLine, 0, xDrawLine, wysokoscOkna);                      //Siatka osi x
                        GraphicRysowanieOsiWspolrzednych.DrawLine(blackPen, xDrawLine, (noweWspOsix - 5), xDrawLine, (noweWspOsix + 5));     //Duza podzialka osi x
                        if (a > -10)
                            GraphicRysowanieOsiWspolrzednych.DrawString(a + "", drawFont1, drawBrush, (xDrawLine - 8), (noweWspOsix + 6));   //Oznaczenia na lewo od osi y
                        else
                            GraphicRysowanieOsiWspolrzednych.DrawString(a + "", drawFont1, drawBrush, (xDrawLine - 12), (noweWspOsix + 6));  //Oznaczenia na lewo od osi y
                    }
                    else if (a == 0)
                        GraphicRysowanieOsiWspolrzednych.DrawString(a + "", drawFont1, drawBrush, (xDrawLine + 4), (noweWspOsix + 6));       //Oznaczenie srodka osi wsp.
                    else if (a > 0)
                    {
                        GraphicRysowanieOsiWspolrzednych.DrawLine(lightgrayPen, xDrawLine, 0, xDrawLine, wysokoscOkna);                      //Siatka osi x
                        GraphicRysowanieOsiWspolrzednych.DrawLine(blackPen, xDrawLine, (noweWspOsix - 5), xDrawLine, (noweWspOsix + 5));     //Duza podzialka osi x
                        if (a < 10)
                            GraphicRysowanieOsiWspolrzednych.DrawString(a + "", drawFont1, drawBrush, (xDrawLine - 4), (noweWspOsix + 6));   //Oznaczenia na lewo od osi y
                        else
                            GraphicRysowanieOsiWspolrzednych.DrawString(a + "", drawFont1, drawBrush, (xDrawLine - 8), (noweWspOsix + 6));   //Oznaczenia na lewo od osi y
                    }
                }

                //Rysowanie podziałki i oznaczeń dla osi y
                for (int i = 0; i < (szerokoscOsiy + 1); i++)       //Mala podzialka osi y (co 0.1)
                {
                    int yDrawLine = Convert.ToInt32(wysokoscOkna - (wysokoscStrzalki + offsetKratkiy + (i * szerokoscKratkiy)));
                    GraphicRysowanieOsiWspolrzednych.DrawLine(blackPen, (noweWspOsiy - 2), yDrawLine, (noweWspOsiy + 2), yDrawLine);
                }
                for (int i = 0; i <= ((szerokoscOsiy / 10)); i++)   //Duza podzialka osi x (co 1.0)
                {
                    int yDrawLine = Convert.ToInt32(wysokoscOkna - (wysokoscStrzalki + offsetKratkiy + (i * (szerokoscKratkiy * 10))));
                    int a = Convert.ToInt32((numy / 10) + i);

                    if (a < 0)
                    {
                        GraphicRysowanieOsiWspolrzednych.DrawLine(lightgrayPen, 0, yDrawLine, szerokoscOkna, yDrawLine);                     //Podzialka osi y
                        GraphicRysowanieOsiWspolrzednych.DrawLine(blackPen, (noweWspOsiy - 5), yDrawLine, (noweWspOsiy + 5), yDrawLine);     //Podzialka osi y
                        GraphicRysowanieOsiWspolrzednych.DrawString(a + "", drawFont1, drawBrush, (noweWspOsiy + 8), (yDrawLine - 8));       //Oznaczenia na lewo od osi x
                    }
                    else if (a > 0)
                    {
                        GraphicRysowanieOsiWspolrzednych.DrawLine(lightgrayPen, 0, yDrawLine, szerokoscOkna, yDrawLine);                     //Podzialka osi y
                        GraphicRysowanieOsiWspolrzednych.DrawLine(blackPen, (noweWspOsiy - 5), yDrawLine, (noweWspOsiy + 5), yDrawLine);     //Podzialka osi y
                        GraphicRysowanieOsiWspolrzednych.DrawString(a + "", drawFont1, drawBrush, (noweWspOsiy + 8), (yDrawLine - 8));       //Oznaczenia na lewo od osi x
                    }
                }

                pictureBox_WykresPolozeniaRobota.Image = bitmapaRysowanieOsiWspolrzednych;
            }
            catch
            {

            }
        }

        public void RysowanieUkladuWspolrzednych(double xWspPkt, double yWspPkt, int fi, int dlugoscWektorax, int dlugoscWektoray, string kolor)
        {
            Pen colorPen = new Pen(Color.Blue, 2);
            Font drawFont = new Font("Arial", 10);
            SolidBrush drawBrush = new SolidBrush(Color.Blue);

            if (kolor == "green")
            {
                colorPen = new Pen(Color.Green, 2);
                drawBrush = new SolidBrush(Color.Green);
            }
            else if (kolor == "red")
            {
                colorPen = new Pen(Color.Red, 2);
                drawBrush = new SolidBrush(Color.Red);
            }
            else if (kolor == "orange")
            {
                colorPen = new Pen(Color.Orange, 2);
                drawBrush = new SolidBrush(Color.Orange);
            }

            int xPkt = 0, yPkt = 0;                     //Wspolrzedne konca wektora dla osi wsp. robota
            int xPktStrzalka = 0, yPktStrzalka = 0;     //Wspolrzedne konca jednej czesci grota strzalki
            int xPktZnak = 0, yPktZnak = 0;             //Wspolrzedne znaku osi x lub y

            int dlugoscWektora = 0;
            double angle = 0.0;
            double angleStrzalka = (Math.PI * 150) / 180;
            //num to wartosci w [dm] od ktorej zaczyna sie podzialka ukl wsp z lewej i od dolu (np. -30dm)
            int wspxKolo = Convert.ToInt32(                wysokoscStrzalki + offsetKratkix + (Convert.ToInt32(Math.Abs(numx - xWspPkt * 10) * (double)szerokoscKratkix)));
            int wspyKolo = Convert.ToInt32(wysokoscOkna - (wysokoscStrzalki + offsetKratkiy + (Convert.ToInt32(Math.Abs(numy - yWspPkt * 10) * (double)szerokoscKratkiy))));

            if (dlugoscWektorax <= dlugoscWektoray) //Wybranie krotszego
                dlugoscWektora = dlugoscWektorax;
            else
                dlugoscWektora = dlugoscWektoray;

            Rectangle pktSrodkoweOsiWsp = new Rectangle((wspxKolo - 4), (wspyKolo - 4), 8, 8);
            GraphicRysowanieOsiWspolrzednych.FillEllipse(drawBrush, pktSrodkoweOsiWsp);

            for (int i = 0; i < 2; i++)
            {
                /*Rysowanie wektora osi x lub y (zalezy od wartosci i)*/
                angle = (Math.PI * (fi + (i * 90))) / 180;    //Zamiana ze stopni na radiany
                xPkt = wspxKolo + Convert.ToInt32(Math.Cos(angle) * (dlugoscWektora * 10));
                yPkt = wspyKolo - Convert.ToInt32(Math.Sin(angle) * (dlugoscWektora * 10));
                GraphicRysowanieOsiWspolrzednych.DrawLine(colorPen, Convert.ToSingle(wspxKolo), Convert.ToSingle(wspyKolo), xPkt, yPkt);     //Rysowanie osi x i y

                /*Przypisanie oznaczenia do osi*/
                xPktZnak = xPkt + Convert.ToInt32(Math.Cos(angle) * (dlugoscWektora * 2));
                yPktZnak = yPkt - Convert.ToInt32(Math.Sin(angle) * (dlugoscWektora * 2));
                if(i == 0)
                    GraphicRysowanieOsiWspolrzednych.DrawString("x", drawFont, drawBrush, xPktZnak, yPktZnak);
                if(i == 1)
                    GraphicRysowanieOsiWspolrzednych.DrawString("y", drawFont, drawBrush, xPktZnak, yPktZnak);

                /*Rysowanie grota strzalki*/
                xPktStrzalka = xPkt + Convert.ToInt32(Math.Cos(angle + angleStrzalka) * (dlugoscWektora * 2));  //Jedna czesc grota
                yPktStrzalka = yPkt - Convert.ToInt32(Math.Sin(angle + angleStrzalka) * (dlugoscWektora * 2));
                GraphicRysowanieOsiWspolrzednych.DrawLine(colorPen, xPkt, yPkt, xPktStrzalka, yPktStrzalka);

                xPktStrzalka = xPkt + Convert.ToInt32(Math.Cos(angle - angleStrzalka) * (dlugoscWektora * 2));  //Druga czesc grota
                yPktStrzalka = yPkt - Convert.ToInt32(Math.Sin(angle - angleStrzalka) * (dlugoscWektora * 2));
                GraphicRysowanieOsiWspolrzednych.DrawLine(colorPen, xPkt, yPkt, xPktStrzalka, yPktStrzalka);
            }
            pictureBox_WykresPolozeniaRobota.Image = bitmapaRysowanieOsiWspolrzednych;
        }

        public void RysowanieSciezkiRobota(double[] xWspPkt, double[] yWspPkt)
        {
            Pen colorPen = new Pen(Color.Orange, 1);
            SolidBrush drawBrush = new SolidBrush(Color.Orange);
            Rectangle pktSrodkoweOsiWsp;

            int wspxKolo = 0;
            int wspyKolo = 0;

            float[] wspXSciezki = new float[xWspPkt.Length];
            float[] wspYSciezki = new float[yWspPkt.Length];
            
            for (int i = 0; i < xWspPkt.Length; i++)
            {
                //Rysowanie punktow w ktorych byl robot
                wspxKolo = Convert.ToInt32(                wysokoscStrzalki + offsetKratkix + (Convert.ToInt32(Math.Abs(numx - xWspPkt[i]*10) * szerokoscKratkix)));
                wspyKolo = Convert.ToInt32(wysokoscOkna - (wysokoscStrzalki + offsetKratkiy + (Convert.ToInt32(Math.Abs(numy - yWspPkt[i]*10) * szerokoscKratkiy))));
                    pktSrodkoweOsiWsp = new Rectangle((wspxKolo - 2), (wspyKolo - 2), 4, 4);
                    GraphicRysowanieOsiWspolrzednych.FillEllipse(drawBrush, pktSrodkoweOsiWsp);

                wspXSciezki[i] = wspxKolo;
                wspYSciezki[i] = wspyKolo;
            }

            for (int i = 1; i < xWspPkt.Length; i++)
            {
                GraphicRysowanieOsiWspolrzednych.DrawLine(colorPen, Convert.ToSingle(wspXSciezki[i - 1]), Convert.ToSingle(wspYSciezki[i - 1]), Convert.ToSingle(wspXSciezki[i]), Convert.ToSingle(wspYSciezki[i]));
            }

            pictureBox_WykresPolozeniaRobota.Image = bitmapaRysowanieOsiWspolrzednych;
        }

        void WyczyscSciezkeRobota()
        {
            Array.Resize(ref tabXActual, 1);
            Array.Resize(ref tabYActual, 1);

            tabXActual[0] = 0.0;
            tabYActual[0] = 0.0;
        }

        void RysowanieOsiPredkosci()
        {
            try
            {
                Bitmap bitmapaPredkoscPrzyspieszenieRobota = new Bitmap(pictureBox_PredkoscPrzyspieszenieRobota.Size.Width, pictureBox_PredkoscPrzyspieszenieRobota.Size.Height);
                Graphics g = Graphics.FromImage(bitmapaPredkoscPrzyspieszenieRobota);
                g.Clear(Color.White);

                int wysokoscOkna = (pictureBox_PredkoscPrzyspieszenieRobota.Size.Height - 1);
                int szerokoscOkna = (pictureBox_PredkoscPrzyspieszenieRobota.Size.Width - 1);
                int szerokoscKratkix = 0, szerokoscKratkiy = 0;
                int wysokoscStrzalki = 15;
                int iloscKratekx = 19, iloscKrateky = 30;

                Font drawFont1 = new Font("Arial", 8);
                Font drawFont2 = new Font("Arial", 10, FontStyle.Regular);
                SolidBrush drawBrush1 = new SolidBrush(Color.Black);
                SolidBrush drawBrush2 = new SolidBrush(Color.OrangeRed);
                SolidBrush drawBrushGreen = new SolidBrush(Color.Green);
                SolidBrush drawBrushRed = new SolidBrush(Color.Red);
                Pen blackPen = new Pen(Color.Black, 1);
                Pen greenPen = new Pen(Color.LimeGreen, 1);
                Pen redPen = new Pen(Color.Red, 1);
                Pen lightgrayPen = new Pen(Color.LightGray, 1);

                szerokoscKratkix = ((szerokoscOkna - (wysokoscStrzalki * 2)) / iloscKratekx);                      //Obliczenie szerokosci kratki (podzialki) w zaleznosci od szerokosci okna i ilosci podzialek (brany pod uwage wynik calkowity!)
                szerokoscKratkiy = ((wysokoscOkna - (wysokoscStrzalki * 2) - 5) / iloscKrateky);                      //Obliczenie szerokosci kratki (podzialki) w zaleznosci od szerokosci okna i ilosci podzialek (brany pod uwage wynik calkowity!)
                tb1.Text = Convert.ToString(szerokoscKratkiy);

                for (int i = 1; i < 20; i++) //Siatka osi x
                {
                    g.DrawLine(lightgrayPen, (wysokoscStrzalki + (i * szerokoscKratkix)), 0, (wysokoscStrzalki + (i * szerokoscKratkix)), (wysokoscOkna - wysokoscStrzalki));
                }
                for (int i = 0; i < 6; i++)  //Siatka osi y
                {
                    g.DrawLine(lightgrayPen, wysokoscStrzalki, (wysokoscOkna - wysokoscStrzalki - (szerokoscKratkiy * 5) - (i * (szerokoscKratkiy * 5))), szerokoscOkna, (wysokoscOkna - wysokoscStrzalki - (szerokoscKratkiy * 5) - (i * (szerokoscKratkiy * 5))));
                }

                if (algorytmStart == true)
                {
                    for (int i = 0; i < 20; i++) //Rysowanie probek predkosci
                    {
                        int probkaX = wysokoscStrzalki + (i * szerokoscKratkix);
                        int probkaY = (wysokoscOkna - wysokoscStrzalki) - Convert.ToInt32((float)((30 * Convert.ToSingle(szerokoscKratkiy)) / 300) * tabSpeedMobileRobot[i]);

                        Rectangle pktPredkosci = new Rectangle((probkaX - 3), (probkaY - 3), 6, 6);
                        g.FillEllipse(drawBrush2, pktPredkosci);
                    }
                }
                //Rysowanie osi wsp. x i y oraz strzalek i ich opisow
                g.DrawLine(blackPen, 0, (wysokoscOkna - wysokoscStrzalki), szerokoscOkna, (wysokoscOkna - wysokoscStrzalki));                             //Os odcietych (x)
                g.DrawLine(blackPen, szerokoscOkna, (wysokoscOkna - wysokoscStrzalki), (szerokoscOkna - 8), ((wysokoscOkna - wysokoscStrzalki) - 3));     //Strzalka dla osi x
                g.DrawLine(blackPen, szerokoscOkna, (wysokoscOkna - wysokoscStrzalki), (szerokoscOkna - 8), ((wysokoscOkna - wysokoscStrzalki) + 3));
                g.DrawString("x[s]", drawFont2, drawBrush1, (szerokoscOkna - 22), (wysokoscOkna - wysokoscStrzalki - 20));                      //Podpis osi
                g.DrawLine(blackPen, wysokoscStrzalki, 0, wysokoscStrzalki, wysokoscOkna);                            //Os rzednych (y)
                g.DrawLine(blackPen, wysokoscStrzalki, 0, (wysokoscStrzalki + 3), 8);                                 //Strzalka dla osi y
                g.DrawLine(blackPen, wysokoscStrzalki, 0, (wysokoscStrzalki - 3), 8);
                g.DrawString("y   [m]", drawFont2, Brushes.Black, 1, -3);                                      //Podpis osi

                for (int i = 0; i < 20; i++) //Rysowanie podzialki osi x
                {
                    g.DrawLine(blackPen, (wysokoscStrzalki + (i * szerokoscKratkix)), (wysokoscOkna - wysokoscStrzalki - 2), (wysokoscStrzalki + (i * szerokoscKratkix)), (wysokoscOkna - wysokoscStrzalki + 2));
                }
                for (int i = 0; i < 3; i++)  //Rysowanie podzialki osi y
                {
                    g.DrawLine(blackPen, (wysokoscStrzalki - 2), (wysokoscOkna - wysokoscStrzalki - (szerokoscKratkiy * 5) - (i * (szerokoscKratkiy * 10))), (wysokoscStrzalki + 2), (wysokoscOkna - wysokoscStrzalki - (szerokoscKratkiy * 5) - (i * (szerokoscKratkiy * 10))));         //Mala podzialka osi y
                    g.DrawLine(blackPen, (wysokoscStrzalki - 4), (wysokoscOkna - wysokoscStrzalki - (szerokoscKratkiy * 10) - (i * (szerokoscKratkiy * 10))), (wysokoscStrzalki + 4), (wysokoscOkna - wysokoscStrzalki - (szerokoscKratkiy * 10) - (i * (szerokoscKratkiy * 10))));     //Duza podzialka osi y
                    g.DrawString((i + 1) + "", drawFont1, drawBrush1, 0, (wysokoscOkna - 7 - wysokoscStrzalki - (szerokoscKratkiy * 10) - (i * (szerokoscKratkiy * 10))));
                }
                g.DrawString("0", drawFont1, drawBrush1, 0, (wysokoscOkna - wysokoscStrzalki + 3));

                pictureBox_PredkoscPrzyspieszenieRobota.Image = bitmapaPredkoscPrzyspieszenieRobota;
            }
            catch
            {

            }
        }

        void WyczyscWykresPredkosci()
        {
            for (int i = 0; i < tabSpeedMobileRobot.Length; i++) { tabSpeedMobileRobot[i] = 0; }
            RysowanieOsiPredkosci();
        }
        
        void RysowanieSiatkiRadaru()
        {
            Bitmap bitmapaRysowanieSiatkiRadaru = new Bitmap(pictureBox_RysunekRadaru.Size.Width, pictureBox_RysunekRadaru.Size.Height);
            GraphicRysunekRadaru = Graphics.FromImage(bitmapaRysowanieSiatkiRadaru);
            GraphicRysunekRadaru.Clear(Color.White);
            GraphicRysunekRadaru.DrawImage(Properties.Resources.Odczyt_z_czujników, new Rectangle(101, 91, 199, 92));

            Font drawFont1 = new Font("Arial", 8);
            Font drawFont2 = new Font("Arial", 7);
            SolidBrush drawBrush = new SolidBrush(Color.Black);
            Pen lightGrayPen = new Pen(Color.LightGray, 1);

            //Kolka pomocnicze do wycentrowania
            /*Rectangle recta = new Rectangle(x_srodekTyl - promienOkregu, y_srodek - promienOkregu, srednicaOkregu, srednicaOkregu);
            Rectangle recta1 = new Rectangle(x_srodekPrzod - promienOkregu, y_srodek - promienOkregu, srednicaOkregu, srednicaOkregu);
            rysunekRadaru.DrawEllipse(redPen, recta);
            rysunekRadaru.DrawEllipse(redPen, recta1);
            */
            
            for (int i = 1; i < 10; i++)
            {
                int a = i * (odstepSiatki / 2);
                Rectangle Arc1 = new Rectangle(x_srodekTyl - promienOkregu - a, y_srodek - promienOkregu - a, srednicaOkregu + (i * odstepSiatki), srednicaOkregu + (i * odstepSiatki));
                Rectangle Arc2 = new Rectangle(x_srodekPrzod - promienOkregu - a, y_srodek - promienOkregu - a, srednicaOkregu + (i * odstepSiatki), srednicaOkregu + (i * odstepSiatki));
                GraphicRysunekRadaru.DrawArc(lightGrayPen, Arc1, 90, 180);
                GraphicRysunekRadaru.DrawArc(lightGrayPen, Arc2, -90, 180);
                GraphicRysunekRadaru.DrawLine(lightGrayPen, x_srodekTyl, y_srodek - promienOkregu - a, 173, y_srodek - promienOkregu - a);
                GraphicRysunekRadaru.DrawLine(lightGrayPen, 215, y_srodek - promienOkregu - a, x_srodekPrzod + i, y_srodek - promienOkregu - a);
                GraphicRysunekRadaru.DrawLine(lightGrayPen, x_srodekTyl, y_srodek + promienOkregu + a, x_srodekPrzod + i, y_srodek + promienOkregu + a);
                GraphicRysunekRadaru.DrawString((i * 10) + "", drawFont1, drawBrush, 175, (y_srodek - promienOkregu - a - 6));
                GraphicRysunekRadaru.DrawString("[cm]", drawFont2, drawBrush, 192, (y_srodek - promienOkregu - a - 5));
            }
            RysowaniePrzeszkody(DaneOdleglosciPrzod);
            RysowaniePrzeszkody(DaneOdleglosciTyl);

            pictureBox_RysunekRadaru.Image = bitmapaRysowanieSiatkiRadaru;
        }

        void RysowaniePrzeszkody(int[] a)
        {
            int szerokoscZnacznika = 40;
            double katAlphaSharp = 0.0;
            float katAlphaUltrasonic = 15;
            double xKatAlpha = 0.0, yKatAlpha = 0.0;

            Font drawFont1 = new Font("Arial", 8);
            SolidBrush drawBrush = new SolidBrush(Color.Black);
            Pen redPen = new Pen(Color.Red, 2);
            Pen greenPen = new Pen(Color.LimeGreen, 2);

            for (int i = 1; i <= 6; i++)
            {
                if (a[0] == 0 && a[i] != 0)
                {
                    Rectangle Arc3 = new Rectangle(x_srodekPrzod - promienOkregu - (a[i]), y_srodek - promienOkregu - (a[i]), srednicaOkregu + (a[i] * 2), srednicaOkregu + (a[i] * 2));
                    //Przeliczenie kata dla luku w zaleznosci od promienia kola tak aby odleglosc miedzy kontem na koncu ramienia trojkata mial podstawe 15*2=30
                    katAlphaSharp = ((szerokoscZnacznika / 2) / (srednicaOkregu + Convert.ToDouble(a[i]) * 2)) * 2 * (180 / 3.141592);
                    if (i != 6)
                    {
                        GraphicRysunekRadaru.DrawArc(redPen, Arc3, (-90 - (Convert.ToInt32(katAlphaSharp) / 2) + ((i - 1) * 45)), Convert.ToInt32(katAlphaSharp));
                    }

                    switch (i)
                    {
                        case 1:
                            GraphicRysunekRadaru.DrawString(a[i] + "", drawFont1, drawBrush, x_srodekPrzod - 8, (y_srodek - promienOkregu - a[i] - 15));
                            break;
                        case 2:
                            xKatAlpha = Math.Cos(90 - (45 * (i - 1))) * (promienOkregu + a[i] + 15);
                            yKatAlpha = Math.Sin(90 - (45 * (i - 1))) * (promienOkregu + a[i] + 5);
                            GraphicRysunekRadaru.DrawString(a[i] + "", drawFont1, drawBrush, Convert.ToInt32(x_srodekPrzod + xKatAlpha), (Convert.ToInt32(y_srodek - yKatAlpha)));
                            break;
                        case 3:
                            GraphicRysunekRadaru.DrawString(a[i] + "", drawFont1, drawBrush, (x_srodekPrzod + promienOkregu + a[i] + 3), y_srodek - 6 + 16);
                            break;
                        case 4:
                            xKatAlpha = Math.Cos(90 - (45 * (i - 1))) * (promienOkregu + a[i] + 15);
                            yKatAlpha = Math.Sin(90 - (45 * (i - 1))) * (promienOkregu + a[i] - 10);
                            GraphicRysunekRadaru.DrawString(a[i] + "", drawFont1, drawBrush, Convert.ToInt32(x_srodekPrzod + xKatAlpha), (Convert.ToInt32(y_srodek - yKatAlpha)));
                            break;
                        case 5:
                            GraphicRysunekRadaru.DrawString(a[i] + "", drawFont1, drawBrush, x_srodekPrzod - 8, (y_srodek + promienOkregu + a[i] + 2));
                            break;
                        case 6:
                            GraphicRysunekRadaru.DrawArc(greenPen, Arc3, (-90 - (Convert.ToInt32(katAlphaUltrasonic) / 2) + ((3 - 1) * 45)), Convert.ToInt32(katAlphaUltrasonic));
                            GraphicRysunekRadaru.DrawString(a[i] + "", drawFont1, drawBrush, (x_srodekPrzod + promienOkregu + a[i] + 3), y_srodek - 6 - 16);
                            break;
                    }
                }
                else if (a[0] == 1 && a[i] != 0)
                {
                    Rectangle Arc3 = new Rectangle(x_srodekTyl - promienOkregu - (a[i]), y_srodek - promienOkregu - (a[i]), srednicaOkregu + a[i] * 2, srednicaOkregu + a[i] * 2);
                    //Przeliczenie kata dla luku w zaleznosci od promienia kola tak aby odleglosc miedzy kontem na koncu ramienia trojkata mial podstawe 15*2=30
                    katAlphaSharp = ((szerokoscZnacznika / 2) / (srednicaOkregu + Convert.ToDouble(a[i]) * 2)) * 2 * (180 / 3.141592);
                    if (i != 6)
                    {
                        GraphicRysunekRadaru.DrawArc(redPen, Arc3, (90 - (Convert.ToInt32(katAlphaSharp) / 2) + ((i - 1) * 45)), Convert.ToInt32(katAlphaSharp));
                    }

                    switch (i)
                    {
                        case 1:
                            GraphicRysunekRadaru.DrawString(a[i] + "", drawFont1, drawBrush, x_srodekTyl - 8, (y_srodek + promienOkregu + a[i] + 2));
                            break;
                        case 2:
                            xKatAlpha = Math.Cos(90 - (45 * (i - 1))) * (promienOkregu + a[i] + 50);
                            yKatAlpha = Math.Sin(90 - (45 * (i - 1))) * (promienOkregu + a[i] - 10);
                            GraphicRysunekRadaru.DrawString(a[i] + "", drawFont1, drawBrush, Convert.ToInt32(x_srodekTyl - xKatAlpha), (Convert.ToInt32(y_srodek + yKatAlpha)));
                            break;
                        case 3:
                            GraphicRysunekRadaru.DrawString(a[i] + "", drawFont1, drawBrush, (x_srodekTyl - promienOkregu - a[i] - 18), y_srodek - 6 + 16);
                            break;
                        case 4:
                            xKatAlpha = Math.Cos(90 - (45 * (i - 1))) * (promienOkregu + a[i] + 50);
                            yKatAlpha = Math.Sin(90 - (45 * (i - 1))) * (promienOkregu + a[i] + 5);
                            GraphicRysunekRadaru.DrawString(a[i] + "", drawFont1, drawBrush, Convert.ToInt32(x_srodekTyl - xKatAlpha), (Convert.ToInt32(y_srodek + yKatAlpha)));
                            break;
                        case 5:
                            GraphicRysunekRadaru.DrawString(a[i] + "", drawFont1, drawBrush, x_srodekTyl - 8, (y_srodek - promienOkregu - a[i] - 15));
                            break;
                        case 6:
                            GraphicRysunekRadaru.DrawArc(greenPen, Arc3, (90 - (Convert.ToInt32(katAlphaUltrasonic) / 2) + ((3 - 1) * 45)), Convert.ToInt32(katAlphaUltrasonic));
                            GraphicRysunekRadaru.DrawString(a[i] + "", drawFont1, drawBrush, (x_srodekTyl - promienOkregu - a[i] - 18), y_srodek - 6 - 16);
                            break;
                    }
                }
            }
        }

        void WyczyscSiatkeRadaru()
        {
            DaneOdleglosciPrzod[0] = 0;
            DaneOdleglosciTyl[0] = 1;

            for (int i = 1; i < 7; i++) { DaneOdleglosciPrzod[i] = 0; DaneOdleglosciTyl[i] = 0; }
            RysowanieSiatkiRadaru();
        }
        
        void RysowanieAutomatuStanu(char typStanu)
        {
            try
            {
                bitmapaOknoZachowanieAlgorytmu = new Bitmap(pictureBox_ZachowanieAlgorytmu.Size.Width, pictureBox_ZachowanieAlgorytmu.Size.Height);
                GraphicOknoZachowanieAlgorytmu = Graphics.FromImage(bitmapaOknoZachowanieAlgorytmu);
                GraphicOknoZachowanieAlgorytmu.Clear(Color.White);

                WyczyscOknoZachowanieAlgorytmu();
                rysunekAutomatuStanu = pictureBox_ZachowanieAlgorytmu.CreateGraphics();

                int wysokoscOknaAS = (pictureBox_ZachowanieAlgorytmu.Size.Height - 1);
                int szerokoscOknaAS = (pictureBox_ZachowanieAlgorytmu.Size.Width - 1);

                int szerokoscKratkixAS = pictureBox_ZachowanieAlgorytmu.Size.Width / 4;
                int offsetKratkixAS = szerokoscKratkixAS / 2;
                int szerokoscKratkiyAS = pictureBox_ZachowanieAlgorytmu.Size.Height / 2;
                int offsetKratkiyAS = szerokoscKratkiyAS / 2; //Domyslnie 100

                int wiersz1AS = offsetKratkiyAS, wiersz2AS = szerokoscKratkiyAS + offsetKratkiyAS;
                int[] kolumnyAS = new int[4] { 0, 0, 0, 0 };

                int rozmiarxKolaAS = 86, rozmiaryKolaAS = 40;

                Font drawFont1 = new Font("Arial", 16);

                Pen blackPen = new Pen(Color.Black, 3);
                Pen blackPen1 = new Pen(Color.Black, 2);
                Pen greenPen = new Pen(Color.LimeGreen, 1);
                Pen lightgrayPen = new Pen(Color.LightGray, 1);
                SolidBrush drawBrushBlack = new SolidBrush(Color.Black);
                SolidBrush drawBrushGreen = new SolidBrush(Color.LimeGreen);
                Rectangle wspStanuStartAS, wspStanuWanderAS, wspStanuAvoidAS, wspStanuGoalAS, wspStanuHaltAS;

                for (int i = 0; i < 4; i++)
                {
                    kolumnyAS[i] = (szerokoscKratkixAS * i) + offsetKratkixAS;
                    //g.DrawLine(lightgrayPen, (szerokoscKratkixAS * i) + offsetKratkixAS, 0, (szerokoscKratkixAS * i) + offsetKratkixAS, wysokoscOknaAS);   //Siatka pomocnicza pionowa
                }

                for (int i = 0; i < 2; i++)
                {
                    //g.DrawLine(lightgrayPen, 0, (szerokoscKratkiyAS * i) + offsetKratkiyAS, szerokoscOknaAS, (szerokoscKratkiyAS * i) + offsetKratkiyAS); //Siatka pomocnicza pozioma
                }

                for (int i = 1; i <= 3; i++)
                {   //Rysowanie przejsc automatu stanu w wierszu pierwszym (S, W, G, H)
                    Rectangle Arc1AS = new Rectangle(szerokoscKratkixAS * i - 40, offsetKratkiyAS - 30, 80, 60);
                    GraphicOknoZachowanieAlgorytmu.DrawArc(blackPen1, Arc1AS, -45, -90);
                    //Strzalka w prawo (1 wiersz)
                    GraphicOknoZachowanieAlgorytmu.DrawLine(blackPen1, kolumnyAS[i] - 24, wiersz1AS - 23, kolumnyAS[i] - 31, wiersz1AS - 32);
                    GraphicOknoZachowanieAlgorytmu.DrawLine(blackPen1, kolumnyAS[i] - 24, wiersz1AS - 23, kolumnyAS[i] - 36, wiersz1AS - 22);
                }

                //Strzalka w lewo (1 wiersz)
                Rectangle Arc2AS = new Rectangle(szerokoscKratkixAS * 2 - 40, offsetKratkiyAS - 30, 80, 60);
                GraphicOknoZachowanieAlgorytmu.DrawArc(blackPen1, Arc2AS, 45, 90);
                GraphicOknoZachowanieAlgorytmu.DrawLine(blackPen1, kolumnyAS[2] - 73, wiersz1AS + 24, kolumnyAS[2] - 65, wiersz1AS + 33);
                GraphicOknoZachowanieAlgorytmu.DrawLine(blackPen1, kolumnyAS[2] - 73, wiersz1AS + 25, kolumnyAS[2] - 60, wiersz1AS + 24);
                //Rysowanie strzałek przejsc z 1 wiersza (z Wander) na 2 wiersz (do Avoid)
                Rectangle Arc3AS = new Rectangle(kolumnyAS[2] - 120, (wiersz1AS - 120 + offsetKratkiyAS - 50), 240, 240);
                Rectangle Arc4AS = new Rectangle(kolumnyAS[0] - 130, (wiersz2AS - 130 - offsetKratkiyAS + 50), 260, 260);
                GraphicOknoZachowanieAlgorytmu.DrawArc(blackPen1, Arc3AS, -193, -30);
                GraphicOknoZachowanieAlgorytmu.DrawArc(blackPen1, Arc4AS, -34, 23);
                //Grot do gory
                GraphicOknoZachowanieAlgorytmu.DrawLine(blackPen1, kolumnyAS[1] - 15, (wiersz1AS + 28 + offsetKratkiyAS - 50), kolumnyAS[1] - 17, (wiersz1AS + 40 + offsetKratkiyAS - 50));
                GraphicOknoZachowanieAlgorytmu.DrawLine(blackPen1, kolumnyAS[1] - 15, (wiersz1AS + 28 + offsetKratkiyAS - 50), kolumnyAS[1] - 7, (wiersz1AS + 38 + offsetKratkiyAS - 50));
                //Grot na dol
                GraphicOknoZachowanieAlgorytmu.DrawLine(blackPen1, kolumnyAS[1] + 26, (wiersz2AS - 25 - offsetKratkiyAS + 50), kolumnyAS[1] + 18, (wiersz2AS - 37 - offsetKratkiyAS + 50));
                GraphicOknoZachowanieAlgorytmu.DrawLine(blackPen1, kolumnyAS[1] + 26, (wiersz2AS - 25 - offsetKratkiyAS + 50), kolumnyAS[1] + 28, (wiersz2AS - 39 - offsetKratkiyAS + 50));
                //Rysowanie strzałek przejsc z 1 wiersza (z Goal) na 2 wiersz (do Avoid)
                Rectangle Arc5AS = new Rectangle(kolumnyAS[3] - 130, (wiersz2AS - 130 - offsetKratkiyAS + 50), 260, 260);
                Rectangle Arc6AS = new Rectangle(kolumnyAS[1] - 120, (wiersz1AS - 120 + offsetKratkiyAS - 50), 240, 240);
                GraphicOknoZachowanieAlgorytmu.DrawArc(blackPen1, Arc5AS, -146, -23);
                GraphicOknoZachowanieAlgorytmu.DrawArc(blackPen1, Arc6AS, 13, 30);
                //Grot do gory
                GraphicOknoZachowanieAlgorytmu.DrawLine(blackPen1, kolumnyAS[2] + 15, (wiersz1AS + 28 + offsetKratkiyAS - 50), kolumnyAS[2] + 17, (wiersz1AS + 40 + offsetKratkiyAS - 50));
                GraphicOknoZachowanieAlgorytmu.DrawLine(blackPen1, kolumnyAS[2] + 15, (wiersz1AS + 28 + offsetKratkiyAS - 50), kolumnyAS[2] + 7, (wiersz1AS + 38 + offsetKratkiyAS - 50));
                //Grot na dol
                GraphicOknoZachowanieAlgorytmu.DrawLine(blackPen1, kolumnyAS[2] - 26, (wiersz2AS - 25 - offsetKratkiyAS + 50), kolumnyAS[2] - 18, (wiersz2AS - 37 - offsetKratkiyAS + 50));
                GraphicOknoZachowanieAlgorytmu.DrawLine(blackPen1, kolumnyAS[2] - 26, (wiersz2AS - 25 - offsetKratkiyAS + 50), kolumnyAS[2] - 28, (wiersz2AS - 39 - offsetKratkiyAS + 50));

                wspStanuStartAS = new Rectangle((kolumnyAS[0] - (rozmiarxKolaAS / 2)), (wiersz1AS - (rozmiaryKolaAS / 2)), rozmiarxKolaAS, rozmiaryKolaAS);
                wspStanuWanderAS = new Rectangle((kolumnyAS[1] - (rozmiarxKolaAS / 2)), (wiersz1AS - (rozmiaryKolaAS / 2)), rozmiarxKolaAS, rozmiaryKolaAS);
                wspStanuAvoidAS = new Rectangle(((szerokoscOknaAS / 2) - (rozmiarxKolaAS / 2)), (wiersz2AS - (rozmiaryKolaAS / 2)), rozmiarxKolaAS, rozmiaryKolaAS);
                wspStanuGoalAS = new Rectangle((kolumnyAS[2] - (rozmiarxKolaAS / 2)), (wiersz1AS - (rozmiaryKolaAS / 2)), rozmiarxKolaAS, rozmiaryKolaAS);
                wspStanuHaltAS = new Rectangle((kolumnyAS[3] - (rozmiarxKolaAS / 2)), (wiersz1AS - (rozmiaryKolaAS / 2)), rozmiarxKolaAS, rozmiaryKolaAS);

                if (typStanu == 'S') { GraphicOknoZachowanieAlgorytmu.FillEllipse(drawBrushGreen, wspStanuStartAS); }   //Wybor aktywnego stanu
                else if (typStanu == 'W') { GraphicOknoZachowanieAlgorytmu.FillEllipse(drawBrushGreen, wspStanuWanderAS); }
                else if (typStanu == 'A') { GraphicOknoZachowanieAlgorytmu.FillEllipse(drawBrushGreen, wspStanuAvoidAS); }
                else if (typStanu == 'G') { GraphicOknoZachowanieAlgorytmu.FillEllipse(drawBrushGreen, wspStanuGoalAS); }
                else if (typStanu == 'H') { GraphicOknoZachowanieAlgorytmu.FillEllipse(drawBrushGreen, wspStanuHaltAS); }

                GraphicOknoZachowanieAlgorytmu.DrawEllipse(blackPen, wspStanuStartAS); GraphicOknoZachowanieAlgorytmu.DrawString("Start", drawFont1, drawBrushBlack, kolumnyAS[0] - 27, wiersz1AS - 11);
                GraphicOknoZachowanieAlgorytmu.DrawEllipse(blackPen, wspStanuWanderAS); GraphicOknoZachowanieAlgorytmu.DrawString("Wander", drawFont1, drawBrushBlack, kolumnyAS[1] - 40, wiersz1AS - 11);
                GraphicOknoZachowanieAlgorytmu.DrawEllipse(blackPen, wspStanuAvoidAS); GraphicOknoZachowanieAlgorytmu.DrawString("Avoid", drawFont1, drawBrushBlack, (szerokoscOknaAS / 2) - 30, wiersz2AS - 11);
                GraphicOknoZachowanieAlgorytmu.DrawEllipse(blackPen, wspStanuGoalAS); GraphicOknoZachowanieAlgorytmu.DrawString("Goal", drawFont1, drawBrushBlack, kolumnyAS[2] - 25, wiersz1AS - 11);
                GraphicOknoZachowanieAlgorytmu.DrawEllipse(blackPen, wspStanuHaltAS); GraphicOknoZachowanieAlgorytmu.DrawString("Halt", drawFont1, drawBrushBlack, kolumnyAS[3] - 25, wiersz1AS - 11);

                pictureBox_ZachowanieAlgorytmu.Image = bitmapaOknoZachowanieAlgorytmu;
            }
            catch
            {

            }
        }

        void RysowanieWektorowStrowan(int[] VM, int[] VD, char typZachowania)
        {
            try
            {
                bitmapaOknoZachowanieAlgorytmu = new Bitmap(pictureBox_ZachowanieAlgorytmu.Size.Width, pictureBox_ZachowanieAlgorytmu.Size.Height);
                GraphicOknoZachowanieAlgorytmu = Graphics.FromImage(bitmapaOknoZachowanieAlgorytmu);
                GraphicOknoZachowanieAlgorytmu.Clear(Color.White);

                int wysokoscOknaWS = pictureBox_ZachowanieAlgorytmu.Size.Height;
                int szerokoscOknaWS = pictureBox_ZachowanieAlgorytmu.Size.Width;
                int srodekOknaXWS = szerokoscOknaWS / 2 - 1;
                int srodekOknaYWS = wysokoscOknaWS / 2 - 1;

                Font drawFont1 = new Font("Arial", 11);
                Pen colorPen = new Pen(Color.Black, 3);
                Pen lightgrayPen = new Pen(Color.LightGray, 1);
                SolidBrush drawBrushBlack = new SolidBrush(Color.Black);
                SolidBrush drawBrush = new SolidBrush(Color.Black);

                //rysowanie osi wsp, szare linie przerywane, przechodzace przez srodek obrazu
                GraphicOknoZachowanieAlgorytmu.DrawLine(lightgrayPen, srodekOknaXWS, 0, srodekOknaXWS, wysokoscOknaWS - 1);
                GraphicOknoZachowanieAlgorytmu.DrawLine(lightgrayPen, 0, srodekOknaYWS, szerokoscOknaWS - 1, srodekOknaYWS);

                string napis = null; double lol = 0;
                int maxValueVectorM = 0;
                for (int i = 0; i < 8; i++) { if (VM[i] > maxValueVectorM) { maxValueVectorM = VM[i]; } } //Wybranie najdluzszego wektora

                if (maxValueVectorM != 0)
                {
                    int maxValuePixForVector = 0;
                    if (srodekOknaXWS > srodekOknaYWS)   //Jesli okno jest szersze niz wyzsze
                    {
                        maxValuePixForVector = srodekOknaYWS; //Wybierz mniejszy wymiar okna
                    }
                    else   //Jesli okno jest wyzsze niz szersze
                    {
                        maxValuePixForVector = srodekOknaXWS;
                    }

                    for (int i = 0; i < 8; i++)
                    {
                        int vectorToPixels = 0;
                        int XpktKoncowyVM = 0, YpktKoncowyVM = 0;
                        int XPktStrzalka1VM = 0, YPktStrzalka1VM = 0;
                        int XPktStrzalka2VM = 0, YPktStrzalka2VM = 0;
                        double angleVM = (Math.PI * VD[i]) / 180;
                        double angleStrzalka1VM = (Math.PI * 240) / 180;
                        double angleStrzalka2VM = (Math.PI * 60) / 180;

                        //Przeliczenie wartosci Vektora M na dlugosc w px
                        vectorToPixels = ((maxValuePixForVector + 1) / maxValueVectorM) * VM[i];

                        XpktKoncowyVM = srodekOknaXWS - Convert.ToInt32(Math.Sin(angleVM) * vectorToPixels);
                        YpktKoncowyVM = srodekOknaYWS - Convert.ToInt32(Math.Cos(angleVM) * vectorToPixels);

                        /*Rysowanie grota strzalki*/
                        int dlugoscGrotaVM = 10;
                        if (VM[i] == 0) { dlugoscGrotaVM = 0; }

                        XPktStrzalka1VM = XpktKoncowyVM + Convert.ToInt32(Math.Cos(angleVM + angleStrzalka1VM) * dlugoscGrotaVM);  //Jedna czesc grota
                        YPktStrzalka1VM = YpktKoncowyVM - Convert.ToInt32(Math.Sin(angleVM + angleStrzalka1VM) * dlugoscGrotaVM);    //jak VM = 0 to 10 na 0 zmienic

                        XPktStrzalka2VM = XpktKoncowyVM + Convert.ToInt32(Math.Cos(angleVM - angleStrzalka2VM) * dlugoscGrotaVM);  //Druga czesc grota
                        YPktStrzalka2VM = YpktKoncowyVM - Convert.ToInt32(Math.Sin(angleVM - angleStrzalka2VM) * dlugoscGrotaVM);

                        if (i == 0)
                        {
                            colorPen = new Pen(Color.Black, 3);
                            drawBrush = new SolidBrush(Color.Black);
                            GraphicOknoZachowanieAlgorytmu.DrawString("Output control", drawFont1, drawBrush, 0, wysokoscOknaWS - 18 - 30);
                            lol = (double)VM[i] / 100;
                            napis = "Vm = " + Convert.ToString(lol) + " [m/s]";
                            GraphicOknoZachowanieAlgorytmu.DrawString(napis, drawFont1, drawBrush, 0, wysokoscOknaWS - 18 - 15);
                            napis = "Vd = " + Convert.ToString(VD[i]) + " [deg]";
                            GraphicOknoZachowanieAlgorytmu.DrawString(napis, drawFont1, drawBrush, 0, wysokoscOknaWS - 18);
                        }

                        if (i == 1)
                        {
                            if (typZachowania == 'w')
                            {
                                colorPen = new Pen(Color.LimeGreen, 3);
                                drawBrush = new SolidBrush(Color.LimeGreen);
                                GraphicOknoZachowanieAlgorytmu.DrawString("Wander", drawFont1, drawBrush, 0, 0);
                                lol = (double)VM[i] / 100;
                                napis = "Vm = " + Convert.ToString(lol) + " [m/s]";
                                GraphicOknoZachowanieAlgorytmu.DrawString(napis, drawFont1, drawBrush, 0, 15);
                                napis = "Vd = " + Convert.ToString(VD[i]) + " [deg]";
                                GraphicOknoZachowanieAlgorytmu.DrawString(napis, drawFont1, drawBrush, 0, 30);
                            }
                            if (typZachowania == 'g')
                            {
                                colorPen = new Pen(Color.Cyan, 3);
                                drawBrush = new SolidBrush(Color.Cyan);
                                GraphicOknoZachowanieAlgorytmu.DrawString("Go to goal", drawFont1, drawBrush, 0, 0);
                                lol = (double)VM[i] / 100;
                                napis = "Vm = " + Convert.ToString(lol) + " [m/s]";
                                GraphicOknoZachowanieAlgorytmu.DrawString(napis, drawFont1, drawBrush, 0, 15);
                                napis = "Vd = " + Convert.ToString(VD[i]) + " [deg]";
                                GraphicOknoZachowanieAlgorytmu.DrawString(napis, drawFont1, drawBrush, 0, 30);
                            }
                        }
                        if (i == 2)
                        {
                            colorPen = new Pen(Color.Red, 3);
                            drawBrush = new SolidBrush(Color.Red);
                            GraphicOknoZachowanieAlgorytmu.DrawString("Avoid obstacle", drawFont1, drawBrush, szerokoscOknaWS - 120, 0);
                            lol = (double)VM[i] / 100;
                            napis = "Vm = " + Convert.ToString(lol) + " [m/s]";
                            GraphicOknoZachowanieAlgorytmu.DrawString(napis, drawFont1, drawBrush, szerokoscOknaWS - 120, 15);
                            napis = "Vd = " + Convert.ToString(VD[2]) + " [deg]";
                            GraphicOknoZachowanieAlgorytmu.DrawString(napis, drawFont1, drawBrush, szerokoscOknaWS - 120, 30);
                        }

                        if (i == 3) { colorPen = new Pen(Color.Lime, 1); }
                        if (i == 4) { colorPen = new Pen(Color.Orange, 1); }
                        if (i == 5) { colorPen = new Pen(Color.Blue, 1); }
                        if (i == 6) { colorPen = new Pen(Color.Magenta, 1); }
                        if (i == 7) { colorPen = new Pen(Color.Cyan, 1); }

                        GraphicOknoZachowanieAlgorytmu.DrawLine(colorPen, srodekOknaXWS, srodekOknaYWS, XpktKoncowyVM, YpktKoncowyVM);
                        GraphicOknoZachowanieAlgorytmu.DrawLine(colorPen, XpktKoncowyVM, YpktKoncowyVM, XPktStrzalka1VM, YPktStrzalka1VM);
                        GraphicOknoZachowanieAlgorytmu.DrawLine(colorPen, XpktKoncowyVM, YpktKoncowyVM, XPktStrzalka2VM, YPktStrzalka2VM);
                    }
                }



                Rectangle Arc1WS = new Rectangle(srodekOknaXWS - 5, srodekOknaYWS - 5, 10, 10);
                GraphicOknoZachowanieAlgorytmu.FillEllipse(drawBrushBlack, Arc1WS);

                pictureBox_ZachowanieAlgorytmu.Image = bitmapaOknoZachowanieAlgorytmu;
            }
            catch
            {

            }
        }

        void WyczyscOknoZachowanieAlgorytmu()
        {
            bitmapaOknoZachowanieAlgorytmu = new Bitmap(pictureBox_ZachowanieAlgorytmu.Size.Width, pictureBox_ZachowanieAlgorytmu.Size.Height);
            GraphicOknoZachowanieAlgorytmu = Graphics.FromImage(bitmapaOknoZachowanieAlgorytmu);
            GraphicOknoZachowanieAlgorytmu.Clear(Color.White);

            //GraphicOknoZachowanieAlgorytmu.FillRectangle(Brushes.Transparent, new Rectangle(0, 0, pictureBox_ZachowanieAlgorytmu.Size.Width, pictureBox_ZachowanieAlgorytmu.Size.Height));

            pictureBox_ZachowanieAlgorytmu.Image = bitmapaOknoZachowanieAlgorytmu;
        }

        void WyczyscWektorySterowan()
        {
            rodzajZachowaniaMS = '0';

            for (int i = 0; i < vectorMagnitude.Length; i++)
            {
                vectorMagnitude[i] = 0;
            }
            for (int i = 0; i < vectorDirection.Length; i++)
            {
                vectorDirection[i] = 0;
            }
        }
        
        private void timer_Pomoc_Tick(object sender, EventArgs e)
        {
            //tb3.Text = Convert.ToString(wyslanoDanePktDoRobota);
        }
        private void button_ControlFromKeyboard_Click(object sender, EventArgs e)
        {
            if (keyboardIsActive == false)
            {
                keyboardIsActive = true;
                Button_ControlFromKeyboard.BackColor = Color.LimeGreen;

                //wl timera do spr klawiatury
            }
            else if (keyboardIsActive == true)
            {
                keyboardIsActive = false;
                Button_ControlFromKeyboard.BackColor = Color.Transparent;

                //wyl tim do spr klawiatury
            }

            /*if ((e.KeyData == Keys.Up) || (e.KeyData == Keys.W))
            {
                Button_MotionForward.BackColor = Color.LimeGreen;
                Button_MotionReverse.BackColor = Color.Transparent;
                Button_MotionLeft.BackColor = Color.Transparent;
                Button_MotionRight.BackColor = Color.Transparent;
                Button_MotionStop.BackColor = Color.Transparent;

                sp.WriteLine("C1"); string LF = string.Join("", '\n'); sp.Write(LF);
                listBox_WypiszDaneOdebrneWyslane("C1", 'w', 'n');
            }
            else if ((e.KeyData == Keys.Down) || (e.KeyData == Keys.S))
            {
                Button_MotionForward.BackColor = Color.Transparent;
                Button_MotionReverse.BackColor = Color.LimeGreen;
                Button_MotionLeft.BackColor = Color.Transparent;
                Button_MotionRight.BackColor = Color.Transparent;
                Button_MotionStop.BackColor = Color.Transparent;

                sp.WriteLine("C0"); string LF = string.Join("", '\n'); sp.Write(LF);
                listBox_WypiszDaneOdebrneWyslane("C0", 'w', 'n');
            }
            else if ((e.KeyData == Keys.Left) || (e.KeyData == Keys.A))
            {
                Button_MotionForward.BackColor = Color.Transparent;
                Button_MotionReverse.BackColor = Color.Transparent;
                Button_MotionLeft.BackColor = Color.LimeGreen;
                Button_MotionRight.BackColor = Color.Transparent;
                Button_MotionStop.BackColor = Color.Transparent;

                sp.WriteLine("C3"); string LF = string.Join("", '\n'); sp.Write(LF);
                listBox_WypiszDaneOdebrneWyslane("C3", 'w', 'n');
            }
            else if ((e.KeyData == Keys.Right) || (e.KeyData == Keys.D))
            {
                Button_MotionForward.BackColor = Color.Transparent;
                Button_MotionReverse.BackColor = Color.Transparent;
                Button_MotionLeft.BackColor = Color.Transparent;
                Button_MotionRight.BackColor = Color.LimeGreen;
                Button_MotionStop.BackColor = Color.Transparent;

                sp.WriteLine("C4"); string LF = string.Join("", '\n'); sp.Write(LF);
                listBox_WypiszDaneOdebrneWyslane("C4", 'w', 'n');
            }
            else
            {
                Button_MotionForward.BackColor = Color.Transparent;
                Button_MotionReverse.BackColor = Color.Transparent;
                Button_MotionLeft.BackColor = Color.Transparent;
                Button_MotionRight.BackColor = Color.Transparent;
                Button_MotionStop.BackColor = Color.Transparent;
            }*/
            /*
            else if (((e.KeyChar == (char)Keys.Up) && (e.KeyChar == (char)Keys.Left)) || ((e.KeyChar == (char)Keys.W) && (e.KeyChar == (char)Keys.A)))
            {

            }
            else if (((e.KeyChar == (char)Keys.Up) && (e.KeyChar == (char)Keys.Right)) || ((e.KeyChar == (char)Keys.W) && (e.KeyChar == (char)Keys.D)))
            {

            }
            else if (((e.KeyChar == (char)Keys.Down) && (e.KeyChar == (char)Keys.Left)) || ((e.KeyChar == (char)Keys.S) && (e.KeyChar == (char)Keys.A)))
            {

            }
            else if (((e.KeyChar == (char)Keys.Down) && (e.KeyChar == (char)Keys.Right)) || ((e.KeyChar == (char)Keys.S) && (e.KeyChar == (char)Keys.D)))
            {

            }*/
        }

        private void Button_TestKamera_Click(object sender, EventArgs e)
        {
            if (Button_TestKamera.Text == "OFF")
            {
                sendDataToMobileRobot("TC1", 'n');

                Button_WyslijNowePoleObiektu.Enabled = true;
                textBox_NowePoleObiektu.Enabled = true;
                Button_TestKamera.Text = "ON";
            }
            else if (Button_TestKamera.Text == "ON")
            {
                sendDataToMobileRobot("TC0", 'n');
                textBox_PoleObiektu.Text = "-";
                Button_WyslijNowePoleObiektu.Enabled = false;
                textBox_NowePoleObiektu.Enabled = false;
                textBox_NowePoleObiektu.Text = "0";
                textBox_NowePoleObiektu.BackColor = Color.White;
                Button_TestKamera.Text = "OFF";
            }
        }
        private void Button_WyslijNowePoleObiektu_Click(object sender, EventArgs e)
        {
            bool blad1 = false;
            textBox_NowePoleObiektu.BackColor = Color.White;

            try
            {
                string rozmiar1 = null, rozmiar2 = null;

                nowePoleObiektu = Convert.ToInt32(textBox_NowePoleObiektu.Text);
                rozmiar1 = Convert.ToString(nowePoleObiektu);
                if(nowePoleObiektu < 10)
                {
                    rozmiar2 = "PDG00000" + rozmiar1;
                    sendDataToMobileRobot(rozmiar2, 'n');
                    MessageBox.Show("Wysłano!", "Wysłano", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
                if (nowePoleObiektu < 100 && nowePoleObiektu >= 10)
                {
                    rozmiar2 = "PDG0000" + rozmiar1;
                    sendDataToMobileRobot(rozmiar2, 'n');
                    MessageBox.Show("Wysłano!", "Wysłano", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
                if (nowePoleObiektu < 1000 && nowePoleObiektu >= 100)
                {
                    rozmiar2 = "PDG000" + rozmiar1;
                    sendDataToMobileRobot(rozmiar2, 'n');
                    MessageBox.Show("Wysłano!", "Wysłano", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
                if (nowePoleObiektu < 10000 && nowePoleObiektu >= 1000)
                {
                    rozmiar2 = "PDG00" + rozmiar1;
                    sendDataToMobileRobot(rozmiar2, 'n');
                    MessageBox.Show("Wysłano!", "Wysłano", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
                if (nowePoleObiektu < 100000 && nowePoleObiektu >= 10000)
                {
                    rozmiar2 = "PDG0" + rozmiar1;
                    sendDataToMobileRobot(rozmiar2, 'n');
                    MessageBox.Show("Wysłano!", "Wysłano", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
                if (nowePoleObiektu < 1000000 && nowePoleObiektu >= 100000)
                {
                    rozmiar2 = "PDG" + rozmiar1;
                    sendDataToMobileRobot(rozmiar2, 'n');
                    MessageBox.Show("Wysłano!", "Wysłano", MessageBoxButtons.OK, MessageBoxIcon.Information);
                }
                if (nowePoleObiektu >= 1000000)
                {
                    textBox_NowePoleObiektu.BackColor = Color.Red;
                    MessageBox.Show("Maksymalna wartość to 999999!", "Złe dane wejściowe", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
            catch
            {
                blad1 = true;
            }

            if(nowePoleObiektu <= 0 || blad1 == true)
            {
                textBox_NowePoleObiektu.BackColor = Color.Red;
                MessageBox.Show("Dozwolony format zapisu to dodatnia liczba całkowita większa od zera!", "Złe dane wejściowe", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }

        }

        private void Button_StartAlgorytm_Click(object sender, EventArgs e)
        {
            Button_WyslijDaneDoRobota.Enabled = false;
            Button_StartAlgorytm.Enabled = false;
            Button_Algorytm1.Enabled = false;
            Button_Algorytm2.Enabled = false;
            Button_Algorytm3.Enabled = false;
            textBox_xPktStart.Enabled = false;
            textBox_yPktStart.Enabled = false;
            textBox_fiPktStart.Enabled = false;
            textBox_xPktStop.Enabled = false;
            textBox_yPktStop.Enabled = false;
            textBox_fiPktStop.Enabled = false;

            Button_StopAlgorytm.Enabled = true;
            Button_StopAlgorytm.Focus();

            nrWspXYActual = 0;
            Array.Resize(ref tabXActual, nrWspXYActual + 1);
            Array.Resize(ref tabYActual, nrWspXYActual + 1);

            tabXActual[0] = xStart;
            tabYActual[0] = yStart;

            timer_SprawdzKomendeUART.Stop();
            sendDataToMobileRobot("PAS1", 'n');
            timer_SprawdzKomendeUART.Start();

            algorytmStart = true;

            WyczyscWykresPredkosci();
            WyczyscSiatkeRadaru();
        }
        private void Button_StopAlgorytm_Click(object sender, EventArgs e)
        {
            sendDataToMobileRobot("PAS0", 'n');

            if (wybranyAlgorytm == '2')
            {
                RysowanieAutomatuStanu('H');
            }

            algorytmStart = false;
            wyslanoDanePktDoRobota = false;

            Button_WyslijDaneDoRobota.Enabled = true;
            Button_Algorytm1.Enabled = true;
            Button_Algorytm2.Enabled = true;
            Button_Algorytm3.Enabled = true;
            textBox_xPktStart.Enabled = true;
            textBox_yPktStart.Enabled = true;
            textBox_fiPktStart.Enabled = true;
            textBox_xPktStop.Enabled = true;
            textBox_yPktStop.Enabled = true;
            textBox_fiPktStop.Enabled = true;

            Button_StartAlgorytm.Enabled = false;
            Button_StopAlgorytm.Enabled = false;

        }

        private void Button_Algorytm1_Click(object sender, EventArgs e)
        {
            Button_Algorytm1.BackColor = Color.LimeGreen;
            Button_Algorytm2.BackColor = Color.Transparent;
            Button_Algorytm3.BackColor = Color.Transparent;

            wybranyAlgorytm = '1';
            
            ClearTextBoxsPktStart();
            ClearTextBoxsPktStop();
            wyslanoDanePktDoRobota = false;

            WyczyscWykresPredkosci();
            WyczyscSiatkeRadaru();
            RysowanieOsiWspolrzednych(xStart, yStart, xStop, yStop);
            WyczyscSciezkeRobota();
            WyczyscOknoZachowanieAlgorytmu();
            groupBox_ZachowanieAlgorytmu.Text = "";
            xActual = 0.0; yActual = 0.0;
            xMaxValue = 0.0; yMaxValue = 0.0; xMinValue = 0.0; yMinValue = 0.0;

            groupBox_DaneStartowe.Visible = true;
            groupBox_DaneKoncowe.Visible = true;
            groupBox_ParamAlgMotorSchemas.Visible = false;
            groupBox_ParamAlgSubsumption.Visible = false;

            Button_WyslijDaneDoRobota.Enabled = true;
            textBox_xPktStart.Enabled = true;
            textBox_yPktStart.Enabled = true;
            textBox_fiPktStart.Enabled = true;
            textBox_xPktStop.Enabled = true;
            textBox_yPktStop.Enabled = true;
            textBox_fiPktStop.Enabled = true;
            Button_StartAlgorytm.Enabled = false;
            Button_StopAlgorytm.Enabled = false;
        }
        private void Button_Algorytm2_Click(object sender, EventArgs e)
        {
            Button_Algorytm1.BackColor = Color.Transparent;
            Button_Algorytm2.BackColor = Color.LimeGreen;
            Button_Algorytm3.BackColor = Color.Transparent;

            wybranyAlgorytm = '2';
            
            ClearTextBoxsPktStart();
            ClearTextBoxsPktStop();
            wyslanoDanePktDoRobota = false;

            WyczyscWykresPredkosci();
            WyczyscSiatkeRadaru();
            RysowanieOsiWspolrzednych(xStart, yStart, xStop, yStop);
            WyczyscSciezkeRobota();
            WyczyscOknoZachowanieAlgorytmu();
            groupBox_ZachowanieAlgorytmu.Text = "";
            xActual = 0.0; yActual = 0.0;
            xMaxValue = 0.0; yMaxValue = 0.0; xMinValue = 0.0; yMinValue = 0.0;

            groupBox_DaneStartowe.Visible = true;
            groupBox_DaneKoncowe.Visible = false;
            groupBox_ParamAlgMotorSchemas.Visible = false;
            groupBox_ParamAlgSubsumption.Visible = true;

            Button_WyslijDaneDoRobota.Enabled = true;
            textBox_xPktStart.Enabled = true;
            textBox_yPktStart.Enabled = true;
            textBox_fiPktStart.Enabled = true;
            textBox_xPktStop.Enabled = false;
            textBox_yPktStop.Enabled = false;
            textBox_fiPktStop.Enabled = false;
            Button_StartAlgorytm.Enabled = false;
            Button_StopAlgorytm.Enabled = false;
        }
        private void Button_Algorytm3_Click(object sender, EventArgs e)
        {
            Button_Algorytm1.BackColor = Color.Transparent;
            Button_Algorytm2.BackColor = Color.Transparent;
            Button_Algorytm3.BackColor = Color.LimeGreen;

            wybranyAlgorytm = '3';
            
            ClearTextBoxsPktStart();
            ClearTextBoxsPktStop();
            wyslanoDanePktDoRobota = false;

            WyczyscWykresPredkosci();
            WyczyscSiatkeRadaru();
            RysowanieOsiWspolrzednych(xStart, yStart, xStop, yStop);
            WyczyscSciezkeRobota();
            WyczyscOknoZachowanieAlgorytmu();
            groupBox_ZachowanieAlgorytmu.Text = "";
            xActual = 0.0; yActual = 0.0;
            xMaxValue = 0.0; yMaxValue = 0.0; xMinValue = 0.0; yMinValue = 0.0;

            groupBox_DaneStartowe.Visible = true;
            groupBox_DaneKoncowe.Visible = false;
            groupBox_ParamAlgMotorSchemas.Visible = true;
            groupBox_ParamAlgSubsumption.Visible = false;

            Button_WyslijDaneDoRobota.Enabled = true;
            textBox_xPktStart.Enabled = true;
            textBox_yPktStart.Enabled = true;
            textBox_fiPktStart.Enabled = true;
            textBox_xPktStop.Enabled = false;
            textBox_yPktStop.Enabled = false;
            textBox_fiPktStop.Enabled = false;
            Button_StartAlgorytm.Enabled = false;
            Button_StopAlgorytm.Enabled = false;
        }

        private void Button_WyslijDaneDoRobota_Click(object sender, EventArgs e)
        {
            bool isError = false;
            bool blad1 = false, blad2 = false, blad3 = false, blad4 = false, blad5 = false;              //blad1 - zle przypadki dla wspolrzednych, blad2 - zle przypadki dla orientacji

            Button_StartAlgorytm.Enabled = false;
            Button_StopAlgorytm.Enabled = false;

            textBox_xPktStart.BackColor = Color.White;      //Kolor tła kolorowany na bialo
            textBox_yPktStart.BackColor = Color.White;      //Wpisane dane sa poprawne
            textBox_fiPktStart.BackColor = Color.White;
            textBox_xPktStop.BackColor = Color.White;
            textBox_yPktStop.BackColor = Color.White;
            textBox_fiPktStop.BackColor = Color.White;

            textBox_AlgSubMaxSpeed.BackColor = Color.White;
            textBox_DistSensActive1.BackColor = Color.White;
            textBox_DistSensActive2.BackColor = Color.White;
            textBox_DistSensActive3.BackColor = Color.White;
            textBox_DistSensActive4.BackColor = Color.White;
            textBox_DistSensActive5.BackColor = Color.White;

            textBox_AlgMotMaxSpeed.BackColor = Color.White;
            textBox_GainAvoid.BackColor = Color.White;
            textBox_GainWander.BackColor = Color.White;
            textBox_GainGoal.BackColor = Color.White;

            //Sprawdzenie poprawnosci wprowadzonych danych
            //GroupBox "Punkt poczatkowy"
            for (int i = 0; i < 3; i++)
            {                    //Sprawdzam czy wpisano w textBox cyfry, jak nie to zaznaczam blad i podnosze flage
                try
                {
                    switch (i)
                    {
                        case 0: xStart = Convert.ToDouble(textBox_xPktStart.Text); break;      //Pobranie ciagu znakow i przekonwertowanie ich
                        case 1: yStart = Convert.ToDouble(textBox_yPktStart.Text); break;
                        case 2: fiStart = Convert.ToInt32(textBox_fiPktStart.Text); break;
                    }
                }
                catch
                {
                    switch (i)
                    {
                        case 0: textBox_xPktStart.BackColor = Color.Red; blad1 = true; break;
                        case 1: textBox_yPktStart.BackColor = Color.Red; blad1 = true; break;
                        case 2: textBox_fiPktStart.BackColor = Color.Red; blad2 = true; break;
                    }
                }
            }

            if (wybranyAlgorytm == '1')
            { //GroupBox "Punkt koncowy"
                for (int i = 0; i < 3; i++)
                {                     //Sprawdzam czy wpisano w textBox cyfry, jak nie to zaznaczam blad i podnosze flage
                    try
                    {
                        switch (i)
                        {
                            case 0: xStop = Convert.ToDouble(textBox_xPktStop.Text); break;
                            case 1: yStop = Convert.ToDouble(textBox_yPktStop.Text); break;
                            case 2: fiStop = Convert.ToInt32(textBox_fiPktStop.Text); break;
                        }
                    }
                    catch
                    {
                        switch (i)
                        {
                            case 0: textBox_xPktStop.BackColor = Color.Red; blad1 = true; break;
                            case 1: textBox_yPktStop.BackColor = Color.Red; blad1 = true; break;
                            case 2: textBox_fiPktStop.BackColor = Color.Red; blad2 = true; break;
                        }
                    }
                }
            }
            if (wybranyAlgorytm == '2')
            { //GroupBox "Parametry algorytmu Subsumption"
                for (int i = 0; i < 6; i++)
                {                     //Sprawdzam czy wpisano w textBox cyfry, jak nie to zaznaczam blad i podnosze flage
                    try
                    {
                        switch (i)
                        {
                            case 0: maxSpeedForMobileRobot = Convert.ToInt16(textBox_AlgSubMaxSpeed.Text); break;
                            case 1: distSensActive1 = Convert.ToInt32(textBox_DistSensActive1.Text); break;
                            case 2: distSensActive2 = Convert.ToInt32(textBox_DistSensActive2.Text); break;
                            case 3: distSensActive3 = Convert.ToInt32(textBox_DistSensActive3.Text); break;
                            case 4: distSensActive4 = Convert.ToInt32(textBox_DistSensActive4.Text); break;
                            case 5: distSensActive5 = Convert.ToInt32(textBox_DistSensActive5.Text); break;
                        }
                    }
                    catch
                    {
                        switch (i)
                        {
                            case 0: textBox_AlgSubMaxSpeed.BackColor = Color.Red; blad3 = true; break;
                            case 1: textBox_DistSensActive1.BackColor = Color.Red; blad4 = true; break;
                            case 2: textBox_DistSensActive2.BackColor = Color.Red; blad4 = true; break;
                            case 3: textBox_DistSensActive3.BackColor = Color.Red; blad4 = true; break;
                            case 4: textBox_DistSensActive4.BackColor = Color.Red; blad4 = true; break;
                            case 5: textBox_DistSensActive5.BackColor = Color.Red; blad4 = true; break;
                        }
                    }
                }
            }
            if (wybranyAlgorytm == '3')
            { //GroupBox "Parametry algorytmu Motor Schemas"
                for (int i = 0; i < 8; i++)
                {                     //Sprawdzam czy wpisano w textBox cyfry, jak nie to zaznaczam blad i podnosze flage
                    try
                    {
                        switch (i)
                        {
                            case 0: maxSpeedForMobileRobot = Convert.ToInt16(textBox_AlgMotMaxSpeed.Text); break;
                            case 1: gainAvoid = Convert.ToDouble(textBox_GainAvoid.Text); break;
                            case 6: gainWander = Convert.ToDouble(textBox_GainWander.Text); break;
                            case 7: gainGoal = Convert.ToDouble(textBox_GainGoal.Text); break;
                        }
                    }
                    catch
                    {
                        switch (i)
                        {
                            case 0: textBox_AlgMotMaxSpeed.BackColor = Color.Red; blad3 = true; break;
                            case 1: textBox_GainAvoid.BackColor = Color.Red; blad5 = true; break;
                            case 6: textBox_GainWander.BackColor = Color.Red; blad5 = true; break;
                            case 7: textBox_GainGoal.BackColor = Color.Red; blad5 = true; break;
                        }
                    }
                }
            }

            if (blad1 == true)      //Jesli wpisano zle dane w polach wspolrzednych x lub y
                MessageBox.Show("Dozwolony format zapisu to 0 lub 0,0!", "Złe dane wejściowe", MessageBoxButtons.OK, MessageBoxIcon.Error);
            if (blad2 == true)      //Jesli wpisano zle dane w polach orientacji
                MessageBox.Show("Dozwolony format zapisu to liczba całkowita od 0 do 359!", "Złe dane wejściowe", MessageBoxButtons.OK, MessageBoxIcon.Error);
            if (blad3 == true)      //Jesli wpisano zle dane w polach maksymalnej predkosci robota
                MessageBox.Show("Dozwolony format zapisu to liczba całkowita od 10 do 250!", "Złe dane wejściowe", MessageBoxButtons.OK, MessageBoxIcon.Error);
            if (blad4 == true)      //Jesli wpisano zle dane w polach aktywacji reakcji sensora
                MessageBox.Show("Dozwolony format zapisu to liczba całkowita od 10 do 80!", "Złe dane wejściowe", MessageBoxButtons.OK, MessageBoxIcon.Error);
            if (blad5 == true)      //Jesli wpisano zle dane w polach wzmocnienia zachowan
                MessageBox.Show("Dozwolony format zapisu to 0 lub 0,0!", "Złe dane wejściowe", MessageBoxButtons.OK, MessageBoxIcon.Error);

            if (blad1 == true || blad2 == true || blad3 == true || blad4 == true || blad5 == true) isError = true;

            if ((xStart == xStop) && (yStart == yStop) && blad1 == false && wybranyAlgorytm == '1')   //Zabezpieczenie przed wpisaniem takich samych wsp startowych i koncowych
            {
                textBox_xPktStart.BackColor = Color.Red;
                textBox_yPktStart.BackColor = Color.Red;
                textBox_xPktStop.BackColor = Color.Red;
                textBox_yPktStop.BackColor = Color.Red;
                MessageBox.Show("Współrzędne punktu początkowego muszą być inne niż współrzędne punktu końcowego!", "Podano złe współrzędne", MessageBoxButtons.OK, MessageBoxIcon.Error);
                isError = true;
            }
            else if ((fiStart < 0 || fiStart > 359) && blad2 == false)      //Jesli wpisano zly zakres stopni dla orientacji poczatkowej
            {
                textBox_fiPktStart.BackColor = Color.Red; isError = true;
                MessageBox.Show("Dozwolony zakres kąta: 0 - 359!", "Niewłaściwy kąt orientacji", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            else if ((fiStop < 0 || fiStop > 359) && blad2 == false && wybranyAlgorytm == '1')        //Jesli wpisano zly zakres stopni dla orientacji koncowej
            {
                textBox_fiPktStop.BackColor = Color.Red; isError = true;
                MessageBox.Show("Dozwolony zakres kąta: 0 - 359!", "Niewłaściwy kąt orientacji", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            else if ((maxSpeedForMobileRobot < 10 || maxSpeedForMobileRobot > 250) && blad3 == false && (wybranyAlgorytm == '2' || wybranyAlgorytm == '3'))
            {
                textBox_AlgSubMaxSpeed.BackColor = Color.Red; isError = true;
                MessageBox.Show("Dozwolona wartość predkości: 10 - 250!", "Niewłaściwa wartość ogrniczenia prędkości", MessageBoxButtons.OK, MessageBoxIcon.Error);
            }
            else if (((distSensActive1 < 10) || (distSensActive1 > 80)) || ((distSensActive2 < 10) || (distSensActive2 > 80)) || ((distSensActive3 < 10) || (distSensActive3 > 80)) || ((distSensActive4 < 10) || (distSensActive4 > 80)) || ((distSensActive5 < 10) || (distSensActive5 > 80)) && blad4 == false && wybranyAlgorytm == '2')
            {
                bool a = false;
                if ((distSensActive1 < 10) || (distSensActive1 > 80)) { textBox_DistSensActive1.BackColor = Color.Red; a = true; }
                if ((distSensActive2 < 10) || (distSensActive2 > 80)) { textBox_DistSensActive2.BackColor = Color.Red; a = true; }
                if ((distSensActive3 < 10) || (distSensActive3 > 80)) { textBox_DistSensActive3.BackColor = Color.Red; a = true; }
                if ((distSensActive4 < 10) || (distSensActive4 > 80)) { textBox_DistSensActive4.BackColor = Color.Red; a = true; }
                if ((distSensActive5 < 10) || (distSensActive5 > 80)) { textBox_DistSensActive5.BackColor = Color.Red; a = true; }
                if (a == true)
                {
                    isError = true;
                    MessageBox.Show("Dozwolona wartość aktywacji reakcji sensora: 10 - 80!", "Niewłaściwa odległość aktywacji reakcji sensora", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
            else if (((gainAvoid < 0.0) || (gainAvoid > 50.0)) || ((gainWander < 0.0) || (gainWander > 50.0)) || ((gainGoal < 0.0) || (gainGoal > 50.0)) && blad5 == false && wybranyAlgorytm == '3')
            {
                bool a = false;
                if ((gainAvoid < 0.0)  || (gainAvoid > 50.0)) { textBox_GainAvoid.BackColor  = Color.Red; a = true; }
                if ((gainWander < 0.0) || (gainWander > 50.0)) { textBox_GainWander.BackColor = Color.Red; a = true; }
                if ((gainGoal < 0.0)   || (gainGoal   > 50.0)) { textBox_GainGoal.BackColor   = Color.Red; a = true; }
                if (a == true)
                {
                    isError = true;
                    MessageBox.Show("Dozwolona wartość wzmocnienia zachowania: 0,0 - 50,0!", "Niewłaściwa wartość wzmocnienia", MessageBoxButtons.OK, MessageBoxIcon.Error);
                }
            }
            if (isError == false)    //Jesli wszystko jest OK to wyslij dane do robota
            {
                //-------------- Zerowanie paska postepu --------------//
                progressBarSendDataToMobileRobot = 0;
                formSendData.progressBar_SendDataToMobileRobot.Value = progressBarSendDataToMobileRobot;

                //-------------- Czszczenie informacji wysylania w label-ach --------------//
                formSendData.label_wybranyAlgorytm.ForeColor = Color.Black; formSendData.label_wybranyAlgorytm.Text = "";

                formSendData.label_StatusXStart.ForeColor = Color.Black; formSendData.label_StatusXStart.Text = "";
                formSendData.label_StatusYStart.ForeColor = Color.Black; formSendData.label_StatusYStart.Text = "";
                formSendData.label_StatusFiStart.ForeColor = Color.Black; formSendData.label_StatusFiStart.Text = "";

                formSendData.label_StatusXStop.ForeColor = Color.Black; formSendData.label_StatusXStop.Text = "";
                formSendData.label_StatusYStop.ForeColor = Color.Black; formSendData.label_StatusYStop.Text = "";
                formSendData.label_StatusFiStop.ForeColor = Color.Black; formSendData.label_StatusFiStop.Text = "";

                formSendData.label_StatusSubMaxV.ForeColor = Color.Black; formSendData.label_StatusSubMaxV.Text = "";
                formSendData.label_StatusDSA1.ForeColor = Color.Black; formSendData.label_StatusDSA1.Text = "";
                formSendData.label_StatusDSA2.ForeColor = Color.Black; formSendData.label_StatusDSA2.Text = "";
                formSendData.label_StatusDSA3.ForeColor = Color.Black; formSendData.label_StatusDSA3.Text = "";
                formSendData.label_StatusDSA4.ForeColor = Color.Black; formSendData.label_StatusDSA4.Text = "";
                formSendData.label_StatusDSA5.ForeColor = Color.Black; formSendData.label_StatusDSA5.Text = "";

                formSendData.label_StatusMotMaxV.ForeColor = Color.Black; formSendData.label_StatusMotMaxV.Text = "";
                formSendData.label_StatusGA.ForeColor = Color.Black; formSendData.label_StatusGA.Text = "";
                formSendData.label_StatusGW.ForeColor = Color.Black; formSendData.label_StatusGW.Text = "";
                formSendData.label_StatusGG.ForeColor = Color.Black; formSendData.label_StatusGG.Text = "";

                //-------------- Wpisanie ktory algorytm jest wybrany --------------//
                formSendData.label_wybranyAlgorytm.ForeColor = Color.Black;
                formSendData.label_wybranyAlgorytm.Text = Convert.ToString(wybranyAlgorytm);

                //-------------- Wyslanie informacji o wybranych algorytmie i wyswietlenie info --------------//
                checkAlgorytm = "PWA" + Convert.ToString(wybranyAlgorytm);
                sendDataToMobileRobot(checkAlgorytm, 'n');

                formSendData.label_StatusWybranyAlgorytm.ForeColor = Color.Orange;
                formSendData.label_StatusWybranyAlgorytm.Text = "Wysyłanie";

                //-------------- Wyslanie informacji o wybranych wsp i orientacji oraz wyswietlenie info --------------//
                xStartWsp = "PSX" + string.Format(US_Culturenfo, "{0:00.00}", xStart);  //Tworzenie formatu ciagu znakow na PSX00.00
                yStartWsp = "PSY" + string.Format(US_Culturenfo, "{0:00.00}", yStart);
                fiStartWsp = "PSO" + string.Format(US_Culturenfo, "{0:000}", fiStart);

                sendDataToMobileRobot(xStartWsp, 'n');
                sendDataToMobileRobot(yStartWsp, 'n');
                sendDataToMobileRobot(fiStartWsp, 'n');

                formSendData.label_StatusXStart.ForeColor = Color.Orange; formSendData.label_StatusXStart.Text = "Wysyłanie";
                formSendData.label_StatusYStart.ForeColor = Color.Orange; formSendData.label_StatusYStart.Text = "Wysyłanie";
                formSendData.label_StatusFiStart.ForeColor = Color.Orange; formSendData.label_StatusFiStart.Text = "Wysyłanie";

                //-------------- Tylko w algorytmie 1 potrzeba pkt koncowego --------------//
                if (wybranyAlgorytm == '1')
                {
                    xStopWsp = "PGX" + string.Format(US_Culturenfo, "{0:00.00}", xStop);
                    yStopWsp = "PGY" + string.Format(US_Culturenfo, "{0:00.00}", yStop);
                    fiStopWsp = "PGO" + string.Format(US_Culturenfo, "{0:000}", fiStop);
                    sendDataToMobileRobot(xStopWsp, 'n');
                    sendDataToMobileRobot(yStopWsp, 'n');
                    sendDataToMobileRobot(fiStopWsp, 'n');

                    formSendData.label_StatusXStop.ForeColor = Color.Orange; formSendData.label_StatusXStop.Text = "Wysyłanie";
                    formSendData.label_StatusYStop.ForeColor = Color.Orange; formSendData.label_StatusYStop.Text = "Wysyłanie";
                    formSendData.label_StatusFiStop.ForeColor = Color.Orange; formSendData.label_StatusFiStop.Text = "Wysyłanie";
                }

                if (wybranyAlgorytm == '2')
                {
                    stringMaxSFMR = "PMV" + string.Format(US_Culturenfo, "{0:000}", maxSpeedForMobileRobot);
                    stringDSA1 = "PSA1" + string.Format(US_Culturenfo, "{0:00}", distSensActive1);
                    stringDSA2 = "PSA2" + string.Format(US_Culturenfo, "{0:00}", distSensActive2);
                    stringDSA3 = "PSA3" + string.Format(US_Culturenfo, "{0:00}", distSensActive3);
                    stringDSA4 = "PSA4" + string.Format(US_Culturenfo, "{0:00}", distSensActive4);
                    stringDSA5 = "PSA5" + string.Format(US_Culturenfo, "{0:00}", distSensActive5);

                    sendDataToMobileRobot(stringMaxSFMR, 'n');
                    sendDataToMobileRobot(stringDSA1, 'n');
                    sendDataToMobileRobot(stringDSA2, 'n');
                    sendDataToMobileRobot(stringDSA3, 'n');
                    sendDataToMobileRobot(stringDSA4, 'n');
                    sendDataToMobileRobot(stringDSA5, 'n');

                    formSendData.label_StatusSubMaxV.ForeColor = Color.Orange; formSendData.label_StatusSubMaxV.Text = "Wysyłanie";
                    formSendData.label_StatusDSA1.ForeColor = Color.Orange; formSendData.label_StatusDSA1.Text = "Wysyłanie";
                    formSendData.label_StatusDSA2.ForeColor = Color.Orange; formSendData.label_StatusDSA2.Text = "Wysyłanie";
                    formSendData.label_StatusDSA3.ForeColor = Color.Orange; formSendData.label_StatusDSA3.Text = "Wysyłanie";
                    formSendData.label_StatusDSA4.ForeColor = Color.Orange; formSendData.label_StatusDSA4.Text = "Wysyłanie";
                    formSendData.label_StatusDSA5.ForeColor = Color.Orange; formSendData.label_StatusDSA5.Text = "Wysyłanie";
                }

                if (wybranyAlgorytm == '3')
                {
                    stringMaxSFMR = "PMV" + string.Format(US_Culturenfo, "{0:000}", maxSpeedForMobileRobot);
                    stringGSA = "PGSA" + string.Format(US_Culturenfo, "{0:000}", (gainAvoid * 10));
                    stringGW = "PGSW" + string.Format(US_Culturenfo, "{0:000}", (gainWander * 10));
                    stringGG = "PGSG" + string.Format(US_Culturenfo, "{0:000}", (gainGoal * 10));

                    sendDataToMobileRobot(stringMaxSFMR, 'n');
                    sendDataToMobileRobot(stringGSA, 'n');
                    sendDataToMobileRobot(stringGW, 'n');
                    sendDataToMobileRobot(stringGG, 'n');

                    formSendData.label_StatusMotMaxV.ForeColor = Color.Orange; formSendData.label_StatusMotMaxV.Text = "Wysyłanie";
                    formSendData.label_StatusGA.ForeColor = Color.Orange; formSendData.label_StatusGA.Text = "Wysyłanie";
                    formSendData.label_StatusGW.ForeColor = Color.Orange; formSendData.label_StatusGW.Text = "Wysyłanie";
                    formSendData.label_StatusGG.ForeColor = Color.Orange; formSendData.label_StatusGG.Text = "Wysyłanie";
                }

                //-------------- WYswietlenie okna z info o wysylaniu danych do robota --------------//

                timer_SprawdzKomendeUART.Stop();
                sendPWA_flag = true;
                sendPSX_flag = true; sendPSY_flag = true; sendPSO_flag = true;
                if (wybranyAlgorytm == '1')
                {
                    sendPGX_flag = true; sendPGY_flag = true; sendPGO_flag = true;
                }
                if (wybranyAlgorytm == '2')
                {
                    sendMSFMR_flag = true;
                    sendDSA1_flag = true; sendDSA2_flag = true; sendDSA3_flag = true; sendDSA4_flag = true; sendDSA5_flag = true;
                }
                if (wybranyAlgorytm == '3')
                {
                    sendMSFMR_flag = true;
                    sendGSA_flag = true;  sendGSW_flag = true; sendGSG_flag = true;
                }
                timer_SprawdzKomendeUART.Start();

                //-------------- Wybranie odpowiedniego okna z danymi wysylanymi w zaleznosci od wybranego algorytmu --------------//
                if (wybranyAlgorytm == '1')
                {
                    formSendData.groupBox3.Visible = true;
                    formSendData.groupBox4.Visible = false;
                    formSendData.groupBox5.Visible = false;
                }
                if (wybranyAlgorytm == '2')
                {
                    formSendData.groupBox3.Visible = false;
                    formSendData.groupBox4.Visible = true;
                    formSendData.groupBox5.Visible = false;
                }
                if (wybranyAlgorytm == '3')
                {
                    formSendData.groupBox3.Visible = false;
                    formSendData.groupBox4.Visible = false;
                    formSendData.groupBox5.Visible = true;
                }

                formSendData.ShowDialog(this);
                formSendData.Button_OK.Enabled = false;
            }
        }

        private void timer_SprawdzKlawiature_Tick(object sender, EventArgs e)
        {
            //Button_MotionForward.Focus();
        }

        private void timer_SprawdzKomendeUART_Tick(object sender, EventArgs e)
        {
            if (sp.IsOpen == true)  //jesli port jest otwarty
            {
                try
                {
                    if (sendPAS_flag == true)
                    {
                        sendDataToMobileRobot("PAS1", 'y');
                    }

                    //Flagi od wyslanych danych punktow i orientacji startowej i koncowej
                    if (sendPWA_flag == true)
                    {
                        formSendData.label_StatusWybranyAlgorytm.ForeColor = Color.Orange; formSendData.label_StatusWybranyAlgorytm.Text = "Wysyłanie";
                        sendDataToMobileRobot(checkAlgorytm, 'y');
                    }
                    if (sendPSX_flag == true)
                    {
                        formSendData.label_StatusXStart.ForeColor = Color.Orange; formSendData.label_StatusXStart.Text = "Wysyłanie";
                        sendDataToMobileRobot(xStartWsp, 'y');
                    }
                    if (sendPSY_flag == true)
                    {
                        formSendData.label_StatusYStart.ForeColor = Color.Orange; formSendData.label_StatusYStart.Text = "Wysyłanie";
                        sendDataToMobileRobot(yStartWsp, 'y');
                    }
                    if (sendPSO_flag == true)
                    {
                        formSendData.label_StatusFiStart.ForeColor = Color.Orange; formSendData.label_StatusFiStart.Text = "Wysyłanie";
                        sendDataToMobileRobot(fiStartWsp, 'y');
                    }
                    if (sendPGX_flag == true)
                    {
                        formSendData.label_StatusXStop.ForeColor = Color.Orange; formSendData.label_StatusXStop.Text = "Wysyłanie";
                        sendDataToMobileRobot(xStopWsp, 'y');
                    }
                    if (sendPGY_flag == true)
                    {
                        formSendData.label_StatusYStop.ForeColor = Color.Orange; formSendData.label_StatusYStop.Text = "Wysyłanie";
                        sendDataToMobileRobot(yStopWsp, 'y');
                    }
                    if (sendPGO_flag == true)
                    {
                        formSendData.label_StatusFiStop.ForeColor = Color.Orange; formSendData.label_StatusFiStop.Text = "Wysyłanie";
                        sendDataToMobileRobot(fiStopWsp, 'y');
                    }

                    //Flagi od wyslanych danych z ograniczeniem predkosci
                    if (sendMSFMR_flag == true)
                    {
                        if (wybranyAlgorytm == '2')
                        {
                            formSendData.label_StatusSubMaxV.ForeColor = Color.Orange; formSendData.label_StatusSubMaxV.Text = "Wysyłanie";
                            sendDataToMobileRobot(stringMaxSFMR, 'y');
                        }
                        if (wybranyAlgorytm == '3')
                        {
                            formSendData.label_StatusSubMaxV.ForeColor = Color.Orange; formSendData.label_StatusSubMaxV.Text = "Wysyłanie";
                            sendDataToMobileRobot(stringMaxSFMR, 'y');
                        }
                    }
                    
                    //Flagi od wyslanych danych z algorytmu Subsumption
                    if (sendDSA1_flag == true)
                    {
                        formSendData.label_StatusDSA1.ForeColor = Color.Orange; formSendData.label_StatusDSA1.Text = "Wysyłanie";
                        sendDataToMobileRobot(stringDSA1, 'y');
                    }
                    if (sendDSA2_flag == true)
                    {
                        formSendData.label_StatusDSA2.ForeColor = Color.Orange; formSendData.label_StatusDSA2.Text = "Wysyłanie";
                        sendDataToMobileRobot(stringDSA2, 'y');
                    }
                    if (sendDSA3_flag == true)
                    {
                        formSendData.label_StatusDSA3.ForeColor = Color.Orange; formSendData.label_StatusDSA3.Text = "Wysyłanie";
                        sendDataToMobileRobot(stringDSA3, 'y');
                    }
                    if (sendDSA4_flag == true)
                    {
                        formSendData.label_StatusDSA4.ForeColor = Color.Orange; formSendData.label_StatusDSA4.Text = "Wysyłanie";
                        sendDataToMobileRobot(stringDSA4, 'y');
                    }
                    if (sendDSA5_flag == true)
                    {
                        formSendData.label_StatusDSA5.ForeColor = Color.Orange; formSendData.label_StatusDSA5.Text = "Wysyłanie";
                        sendDataToMobileRobot(stringDSA5, 'y');
                    }

                    //Flagi od wyslanych danych z algorytmu Motor Schemas
                    if (sendGSA_flag == true)
                    {
                        formSendData.label_StatusGA.ForeColor = Color.Orange; formSendData.label_StatusGA.Text = "Wysyłanie";
                        sendDataToMobileRobot(stringGSA, 'y');
                    }
                    if (sendGSW_flag == true)
                    {
                        formSendData.label_StatusGW.ForeColor = Color.Orange; formSendData.label_StatusGW.Text = "Wysyłanie";
                        sendDataToMobileRobot(stringGW, 'y');
                    }
                    if (sendGSG_flag == true)
                    {
                        formSendData.label_StatusGG.ForeColor = Color.Orange; formSendData.label_StatusGG.Text = "Wysyłanie";
                        sendDataToMobileRobot(stringGG, 'y');
                    }
                    
                    //Flagi od skrzyn biegow
                    if (sendGM_flag == true)     //Jesli nie bylo potwierdzenia zostaw jak bylo
                    {
                        sendGM_flag = false;
                        Button_GearManual.Enabled = true;
                        Button_GearAutomatic.Enabled = true;
                    }
                    if (sendGA_flag == true)
                    {
                        sendGA_flag = false;
                        Button_GearManual.Enabled = true;
                        Button_GearAutomatic.Enabled = true;
                        AllGearEnable_Button();
                    }
                    if (sendN_flag  == true)
                    {
                        sendN_flag = false;

                        Button_GearManual.Enabled = true;
                        Button_GearAutomatic.Enabled = true;
                        AllGearEnable_Button();
                    }
                    if (send1_flag  == true)
                    {
                        send1_flag = false;

                        Button_GearManual.Enabled = true;
                        Button_GearAutomatic.Enabled = true;
                        AllGearEnable_Button();
                    }
                    if (send2_flag  == true)
                    {
                        send2_flag = false;

                        Button_GearManual.Enabled = true;
                        Button_GearAutomatic.Enabled = true;
                        AllGearEnable_Button();
                    }
                    if (send3_flag  == true)
                    {
                        send3_flag = false;

                        Button_GearManual.Enabled = true;
                        Button_GearAutomatic.Enabled = true;
                        AllGearEnable_Button();
                    }
                    if (send4_flag  == true)
                    {
                        send4_flag = false;

                        Button_GearManual.Enabled = true;
                        Button_GearAutomatic.Enabled = true;
                        AllGearEnable_Button();
                    }
                    if (send5_flag  == true)
                    {
                        send5_flag = false;

                        Button_GearManual.Enabled = true;
                        Button_GearAutomatic.Enabled = true;
                        AllGearEnable_Button();
                    }
                    if (send6_flag  == true)
                    {
                        send6_flag = false;

                        Button_GearManual.Enabled = true;
                        Button_GearAutomatic.Enabled = true;
                        AllGearEnable_Button();
                    }
                    if (sendR_flag  == true)
                    {
                        sendR_flag = false;

                        Button_GearManual.Enabled = true;
                        Button_GearAutomatic.Enabled = true;
                        AllGearEnable_Button();
                    }
                    
                    //Flagi od oswietlenia
                    if (sendL0_flag == true)
                    {
                        if (StanSwiatlaDzienne == true)
                        {
                            CheckBox_SwiatlaDzienne.Invoke(new Action(delegate () { CheckBox_SwiatlaDzienne.Checked = true; }));
                        }
                        if (StanSwiatlaDzienne == false)
                        {
                            CheckBox_SwiatlaDzienne.Invoke(new Action(delegate () { CheckBox_SwiatlaDzienne.Checked = false; }));
                        }
                        AllLightEnable_CheckBox();

                        sendL0_flag = false;
                    }
                    if (sendL1_flag == true)
                    {
                        if (StanSwiatlaPostojowe == true)
                        {
                            CheckBox_SwiatlaPostojowe.Invoke(new Action(delegate () { CheckBox_SwiatlaPostojowe.Checked = true; }));
                        }
                        if (StanSwiatlaPostojowe == false)
                        {
                            CheckBox_SwiatlaPostojowe.Invoke(new Action(delegate () { CheckBox_SwiatlaPostojowe.Checked = false; }));
                        }
                        AllLightEnable_CheckBox();

                        sendL1_flag = false;
                    }
                    if (sendL2_flag == true)
                    {
                        if (StanSwiatlaKrotkie == true)
                        {
                            CheckBox_SwiatlaKrotkie.Invoke(new Action(delegate () { CheckBox_SwiatlaKrotkie.Checked = true; }));
                        }
                        if (StanSwiatlaKrotkie == false)
                        {
                            CheckBox_SwiatlaKrotkie.Invoke(new Action(delegate () { CheckBox_SwiatlaKrotkie.Checked = false; }));
                        }
                        AllLightEnable_CheckBox();

                        sendL2_flag = false;
                    }
                    if (sendL3_flag == true)
                    {
                        if (StanSwiatlaDlugie == true)
                        {
                            CheckBox_SwiatlaDlugie.Invoke(new Action(delegate () { CheckBox_SwiatlaDlugie.Checked = true; }));
                        }
                        if (StanSwiatlaDlugie == false)
                        {
                            CheckBox_SwiatlaDlugie.Invoke(new Action(delegate () { CheckBox_SwiatlaDlugie.Checked = false; }));
                        }
                        AllLightEnable_CheckBox();

                        sendL3_flag = false;
                    }
                  //if (sendL4_flag == true)
                    if (sendL5_flag == true)
                    {
                        if (StanSwiatlaPrzeciwmgloweTylne == true)
                        {
                            CheckBox_SwiatlaPrzeciwmgloweTylne.Invoke(new Action(delegate () { CheckBox_SwiatlaPrzeciwmgloweTylne.Checked = true; }));
                        }
                        if (StanSwiatlaPrzeciwmgloweTylne == false)
                        {
                            CheckBox_SwiatlaPrzeciwmgloweTylne.Invoke(new Action(delegate () { CheckBox_SwiatlaPrzeciwmgloweTylne.Checked = false; }));
                        }
                        AllLightEnable_CheckBox();

                        sendL5_flag = false;
                    }
                    if (sendL6_flag == true)
                    {
                        if (StanSwiatlaAwaryjne == true)
                        {
                            CheckBox_SwiatlaAwaryjne.Invoke(new Action(delegate () { CheckBox_SwiatlaAwaryjne.Checked = true; }));
                        }
                        if (StanSwiatlaAwaryjne == false)
                        {
                            CheckBox_SwiatlaAwaryjne.Invoke(new Action(delegate () { CheckBox_SwiatlaAwaryjne.Checked = false; }));
                        }
                        AllLightEnable_CheckBox();

                        sendL6_flag = false;
                    }
                    if (sendL7_flag == true)
                    {
                        if (StanMigaczLewy == true)
                        {
                            CheckBox_MigaczLewy.Invoke(new Action(delegate () { CheckBox_MigaczLewy.Checked = true; }));
                        }
                        if (StanMigaczLewy == false)
                        {
                            CheckBox_MigaczLewy.Invoke(new Action(delegate () { CheckBox_MigaczLewy.Checked = false; }));
                        }
                        AllLightEnable_CheckBox();

                        sendL7_flag = false;
                    }
                    if (sendL8_flag == true)
                    {
                        if (StanMigaczPrawy == true)
                        {
                            CheckBox_MigaczPrawy.Invoke(new Action(delegate () { CheckBox_MigaczPrawy.Checked = true; }));
                        }
                        if (StanMigaczPrawy == false)
                        {
                            CheckBox_MigaczPrawy.Invoke(new Action(delegate () { CheckBox_MigaczPrawy.Checked = false; }));
                        }
                        AllLightEnable_CheckBox();

                        sendL8_flag = false;
                    }
                    if (sendL9_flag == true)
                    {
                        if (StanKlakson == true)
                        {
                            CheckBox_Klakson.Invoke(new Action(delegate () { CheckBox_Klakson.Checked = true; }));
                        }
                        if (StanKlakson == false)
                        {
                            CheckBox_Klakson.Invoke(new Action(delegate () { CheckBox_Klakson.Checked = false; }));
                        }
                        AllLightEnable_CheckBox();

                        sendL9_flag = false;
                    }
                    
                }
                catch { }
            }
        }

        public void listBox_WypiszDaneOdebrneWyslane(string data, char typeReadWrite, char repeatOrError)
        {
            if(typeReadWrite == 'w')    //Wyslanie
            {
                if(repeatOrError == 'n')
                {
                    listBox_UARTReceivedSendData.Invoke(new Action(delegate ()
                    {
                        listBox_UARTReceivedSendData.Items.Add("Wysłano: " + data);
                        if (checkBox_AutoScroll.Checked == true)
                        {
                            listBox_UARTReceivedSendData.SelectedIndex = listBox_UARTReceivedSendData.Items.Count - 1;
                            listBox_UARTReceivedSendData.SelectedIndex = -1;
                        }
                    }));
                }
                if (repeatOrError == 'y')
                {
                    listBox_UARTReceivedSendData.Invoke(new Action(delegate ()
                    {
                        listBox_UARTReceivedSendData.Items.Add("Wysłano: " + data + "      (Ponowne wysłanie)");
                        if (checkBox_AutoScroll.Checked == true)
                        {
                            listBox_UARTReceivedSendData.SelectedIndex = listBox_UARTReceivedSendData.Items.Count - 1;
                            listBox_UARTReceivedSendData.SelectedIndex = -1;
                        }
                    }));
                }
                if (repeatOrError == 'e')
                {
                    listBox_UARTReceivedSendData.Invoke(new Action(delegate ()
                    {
                        listBox_UARTReceivedSendData.Items.Add("Błąd wysyłania: " + data);
                        if (checkBox_AutoScroll.Checked == true)
                        {
                            listBox_UARTReceivedSendData.SelectedIndex = listBox_UARTReceivedSendData.Items.Count - 1;
                            listBox_UARTReceivedSendData.SelectedIndex = -1;
                        }
                    }));
                }
            }
            if (typeReadWrite == 'r')   //Odczytanie
            {
                listBox_UARTReceivedSendData.Invoke(new Action(delegate ()  //wywolanie listBox z metody w ktorej zostala ona utworzona, bez tego bedzie blad
                {
                    listBox_UARTReceivedSendData.Items.Add("Odebrano: " + data);
                    if (checkBox_AutoScroll.Checked == true)
                    {
                        listBox_UARTReceivedSendData.SelectedIndex = listBox_UARTReceivedSendData.Items.Count - 1;
                        listBox_UARTReceivedSendData.SelectedIndex = -1;
                    }
                }));
            }
        }

        private void timer_OdbierzDaneUART_Tick(object sender, EventArgs e) //Cyklicznie sprawdzaj co przyszlo
        {
            if (sp.IsOpen == true)  //jesli port jest otwarty
            {
                try
                {
                    newReadDataBuffor = readDataBuffor;   //Przypisanie bufora do dalszej obrobki
                    readDataBuffor = null;                  //Wyczyszczenie bufora
                    
                    if (newReadDataBuffor != null)
                    {
                        newReadDataBuffor = oldReadDataBuffor + newReadDataBuffor;
                        oldReadDataBuffor = null;

                        string tabBezNull = null;

                        for (int i = 0; i < newReadDataBuffor.Length; i++)  //Wywalanie znakow '\0' z bufora bo nie wyswietla przez to wszystkiego w listBox
                        {
                            if(newReadDataBuffor[i] != '\0')    //rozne od null
                            {
                                tabBezNull += newReadDataBuffor[i];
                            }
                        }

                        newReadDataBuffor = tabBezNull;

                        if (checkBox_ShowReceivedBuff.Checked == true)
                        {
                            listBox_UARTReceivedSendData.Invoke(new Action(delegate ()
                            {
                                listBox_UARTReceivedSendData.Items.Add(" ");
                                listBox_UARTReceivedSendData.Items.Add(Convert.ToString("Bufor odbiorczy: ") + newReadDataBuffor);
                                listBox_UARTReceivedSendData.Items.Add(" ");
                                if (checkBox_AutoScroll.Checked == true)
                                {
                                    listBox_UARTReceivedSendData.SelectedIndex = listBox_UARTReceivedSendData.Items.Count - 1;
                                    listBox_UARTReceivedSendData.SelectedIndex = -1;
                                }
                            }));
                        }

                        var inDataChar = newReadDataBuffor.ToCharArray();   //Zapis odebranych danych do tablicy znakow (char)
                        char[] TabRozkazow = new char[1];
                        int rozmiarTabRozkazow = 0;
                        bool startRamki = false;
                        //listBox_UARTReceivedSendData.Invoke(new Action(delegate ()
                        //{
                            //listBox_UARTReceivedSendData.Items.Add(Convert.ToString(inDataChar.Length));
                        //    listBox_UARTReceivedSendData.Items.Add(Convert.ToString(checkReadDataBuffor));
                        //}));

                        for (int i = 0; i < inDataChar.Length; i++) //Sprawdzam wszystkie odebrane rozkazy
                        {
                            if (inDataChar[i] == '!')   //Jesli poczatek rozkazu
                            {
                                startRamki = true;
                            }

                            if (inDataChar[i] == 10)   //Jesli koniec rozkazu '\n'
                            {
                                startRamki = false;

                                for(int j = 0; j < TabRozkazow.Length-1; j++)
                                {
                                    TabRozkazow[j] = TabRozkazow[j + 1];
                                }

                                Array.Resize(ref TabRozkazow, rozmiarTabRozkazow);
                                string s = string.Join("", TabRozkazow);
                                listBox_WypiszDaneOdebrneWyslane(s, 'r', 'n');

                                if (TabRozkazow.Length > 0)
                                {
                                    Komendy(TabRozkazow); //funkcja dekodujaca komendy
                                }
                                rozmiarTabRozkazow = 0;
                                Array.Resize(ref TabRozkazow, (rozmiarTabRozkazow + 1));
                            }
                            if(startRamki == true)
                            {
                                //if (inDataChar[i] != '?' || inDataChar[i] != '@')
                                //{
                                    TabRozkazow[rozmiarTabRozkazow] = inDataChar[i];            //Przypisanie znaku do tablicy ktora bedzie przechowywac rozkaz

                                    if (i == (inDataChar.Length - 1)) //Zapis komendy do tablicy ktora zostala ucieta i ktora nalezy dodac na top przy ponownym wejsciu fora
                                    {
                                        oldReadDataBuffor = string.Join("", TabRozkazow);
                                    }

                                    ++rozmiarTabRozkazow;                                       //Zwiekszenie zmiennej okreslajacej rozmiar tab rozkazu
                                    Array.Resize(ref TabRozkazow, (rozmiarTabRozkazow + 1));    //Powiekszenie tab rozkazu o jeden zeby mozna bylo przyjac kolejny znak
                                //}
                            }
                        }
                    }
                }
                catch { }
            }
        }

        void Komendy(char[] komenda)
        {
            switch (komenda[0])
            {
                case 'G':
                    switch (komenda[1])
                    {
                        case 'M':
                            if ((komenda[2] == '1') && (sendGM_flag == true))
                            {
                                sendGM_flag = false;

                                Button_GearManual.BackColor = Color.LimeGreen;
                                Button_GearManual.Invoke(new Action(delegate () { Button_GearManual.Enabled = false; }));
                                Button_GearAutomatic.BackColor = Color.Transparent;
                                Button_GearAutomatic.Invoke(new Action(delegate () { Button_GearAutomatic.Enabled = true; }));

                                AllGearEnable_Button();
                                ActiveGear = 'N';
                                Button_GearN.BackColor = Color.LimeGreen;
                                Button_GearN.Invoke(new Action(delegate () { Button_GearN.Enabled = false; }));
                                Button_ControlFromKeyboard.Invoke(new Action(delegate () { Button_ControlFromKeyboard.Enabled = true; }));
                            }
                            break;

                        case 'A':
                            if ((komenda[2] == '1') && (sendGA_flag == true))
                            {
                                sendGA_flag = false;

                                Button_GearManual.BackColor = Color.Transparent;
                                Button_GearManual.Invoke(new Action(delegate () { Button_GearManual.Enabled = true; }));
                                Button_GearAutomatic.BackColor = Color.LimeGreen;
                                Button_GearAutomatic.Invoke(new Action(delegate () { Button_GearAutomatic.Enabled = false; }));

                                AllGearDisable_Button();
                                ClearGearBackColor();
                                Button_ControlFromKeyboard.Invoke(new Action(delegate () { Button_ControlFromKeyboard.Enabled = true; }));
                            }
                            break;

                        case 'N':
                            if ((komenda[2] == '1') && (sendN_flag == true))
                            {
                                sendN_flag = false;

                                sp.Write("C0"); string LF = string.Join("", '\n'); sp.Write(LF);
                                listBox_WypiszDaneOdebrneWyslane("C0", 'w', 'n');

                                ActiveGear = 'N';
                                Button_GearAutomatic.Invoke(new Action(delegate () { Button_GearAutomatic.Enabled = true; }));
                                SelectActiveGearBackColor(ActiveGear);
                            }
                            break;

                        case '1':
                            if ((komenda[2] == '1') && (send1_flag == true))
                            {
                                send1_flag = false;

                                ActiveGear = '1';
                                Button_GearAutomatic.Invoke(new Action(delegate () { Button_GearAutomatic.Enabled = true; }));
                                SelectActiveGearBackColor(ActiveGear);
                            }
                            break;

                        case '2':
                            if ((komenda[2] == '1') && (send2_flag == true))
                            {
                                send2_flag = false;

                                ActiveGear = '2';
                                Button_GearAutomatic.Invoke(new Action(delegate () { Button_GearAutomatic.Enabled = true; }));
                                SelectActiveGearBackColor(ActiveGear);
                            }
                            break;

                        case '3':
                            if ((komenda[2] == '1') && (send3_flag == true))
                            {
                                send3_flag = false;

                                ActiveGear = '3';
                                Button_GearAutomatic.Invoke(new Action(delegate () { Button_GearAutomatic.Enabled = true; }));
                                SelectActiveGearBackColor(ActiveGear);
                            }
                            break;

                        case '4':
                            if ((komenda[2] == '1') && (send4_flag == true))
                            {
                                send4_flag = false;

                                ActiveGear = '4';
                                Button_GearAutomatic.Invoke(new Action(delegate () { Button_GearAutomatic.Enabled = true; }));
                                SelectActiveGearBackColor(ActiveGear);
                            }
                            break;

                        case '5':
                            if ((komenda[2] == '1') && (send5_flag == true))
                            {
                                send5_flag = false;

                                ActiveGear = '5';
                                Button_GearAutomatic.Invoke(new Action(delegate () { Button_GearAutomatic.Enabled = true; }));
                                SelectActiveGearBackColor(ActiveGear);
                            }
                            break;

                        case '6':
                            if ((komenda[2] == '1') && (send6_flag == true))
                            {
                                send6_flag = false;

                                ActiveGear = '6';
                                Button_GearAutomatic.Invoke(new Action(delegate () { Button_GearAutomatic.Enabled = true; }));
                                SelectActiveGearBackColor(ActiveGear);
                            }
                            break;

                        case 'R':
                            if ((komenda[2] == '1') && (sendR_flag == true))
                            {
                                sendR_flag = false;

                                sp.Write("C0"); string LF = string.Join("", '\n'); sp.Write(LF);
                                listBox_WypiszDaneOdebrneWyslane("C0", 'w', 'n');
                                ActiveGear = 'R';
                                Button_GearAutomatic.Invoke(new Action(delegate () { Button_GearAutomatic.Enabled = true; }));
                                SelectActiveGearBackColor(ActiveGear);
                            }
                            break;
                    }
                break;

                case 'L':
                    switch (komenda[1])
                    {
                        case '0':
                            if ((komenda[2] == '1') && (sendL0_flag == true))
                            {
                                pictureBox_SwiatlaDzienne.Image = Properties.Resources._004_light_green;
                                StanSwiatlaDzienne = true;
                                CheckBox_SwiatlaDzienne.Invoke(new Action(delegate () { CheckBox_SwiatlaDzienne.Checked = true; }));

                                AllLightEnable_CheckBox();

                                sendL0_flag = false;
                            }
                            if ((komenda[2] == '0') && (sendL0_flag == true))
                            {
                                pictureBox_SwiatlaDzienne.Image = Properties.Resources._004_light;
                                StanSwiatlaDzienne = false;
                                CheckBox_SwiatlaDzienne.Invoke(new Action(delegate () { CheckBox_SwiatlaDzienne.Checked = false; }));

                                AllLightEnable_CheckBox();

                                sendL0_flag = false;
                            }
                        break;

                        case '1':
                            if ((komenda[2] == '1') && (sendL1_flag == true))
                            {
                                pictureBox_SwiatlaPostojowe.Image = Properties.Resources._006_parking_lights_green;
                                StanSwiatlaPostojowe = true;
                                CheckBox_SwiatlaPostojowe.Invoke(new Action(delegate () { CheckBox_SwiatlaPostojowe.Checked = true; }));

                                AllLightEnable_CheckBox();

                                sendL1_flag = false;
                            }
                            if ((komenda[2] == '0') && (sendL1_flag == true))
                            {
                                pictureBox_SwiatlaPostojowe.Image = Properties.Resources._006_parking_lights;
                                StanSwiatlaPostojowe = false;
                                CheckBox_SwiatlaPostojowe.Invoke(new Action(delegate () { CheckBox_SwiatlaPostojowe.Checked = false; }));

                                AllLightEnable_CheckBox();

                                sendL1_flag = false;
                            }
                        break;

                        case '2':
                            if ((komenda[2] == '1') && (sendL2_flag == true))
                            {
                                pictureBox_SwiatlaKrotkie.Image = Properties.Resources._009_low_beam_green;
                                StanSwiatlaKrotkie = true;
                                CheckBox_SwiatlaKrotkie.Invoke(new Action(delegate () { CheckBox_SwiatlaKrotkie.Checked = true; }));

                                AllLightEnable_CheckBox();

                                sendL2_flag = false;
                            }
                            if ((komenda[2] == '0') && (sendL2_flag == true))
                            {
                                pictureBox_SwiatlaKrotkie.Image = Properties.Resources._009_low_beam;
                                StanSwiatlaKrotkie = false;
                                CheckBox_SwiatlaKrotkie.Invoke(new Action(delegate () { CheckBox_SwiatlaKrotkie.Checked = false; }));

                                AllLightEnable_CheckBox();

                                sendL2_flag = false;
                            }
                        break;

                        case '3':
                            if ((komenda[2] == '1') && (sendL3_flag == true))
                            {
                                pictureBox_SwiatlaDlugie.Image = Properties.Resources._012_high_beam_blue;
                                StanSwiatlaDlugie = true;
                                CheckBox_SwiatlaDlugie.Invoke(new Action(delegate () { CheckBox_SwiatlaDlugie.Checked = true; }));

                                AllLightEnable_CheckBox();

                                sendL3_flag = false;
                            }
                            if ((komenda[2] == '0') && (sendL3_flag == true))
                            {
                                pictureBox_SwiatlaDlugie.Image = Properties.Resources._012_high_beam;
                                StanSwiatlaDlugie = false;
                                CheckBox_SwiatlaDlugie.Invoke(new Action(delegate () { CheckBox_SwiatlaDlugie.Checked = false; }));

                                AllLightEnable_CheckBox();

                                sendL3_flag = false;
                            }
                        break;

                        case '4':

                        break;

                        case '5':
                            if ((komenda[2] == '1') && (sendL5_flag == true))
                            {
                                pictureBox_SwiatlaPrzeciwmgloweTylne.Image = Properties.Resources._016_rear_fog_lamp_50_orange;
                                StanSwiatlaPrzeciwmgloweTylne = true;
                                CheckBox_SwiatlaPrzeciwmgloweTylne.Invoke(new Action(delegate () { CheckBox_SwiatlaPrzeciwmgloweTylne.Checked = true; }));

                                AllLightEnable_CheckBox();

                                sendL5_flag = false;
                            }
                            if ((komenda[2] == '0') && (sendL5_flag == true))
                            {
                                pictureBox_SwiatlaPrzeciwmgloweTylne.Image = Properties.Resources._016_rear_fog_lamp_50;
                                StanSwiatlaPrzeciwmgloweTylne = false;
                                CheckBox_SwiatlaPrzeciwmgloweTylne.Invoke(new Action(delegate () { CheckBox_SwiatlaPrzeciwmgloweTylne.Checked = false; }));

                                AllLightEnable_CheckBox();

                                sendL5_flag = false;
                            }
                        break;

                        case '6':
                            if ((komenda[2] == '1') && (sendL6_flag == true))
                            {
                                CheckBox_MigaczLewy.Invoke(new Action(delegate () { CheckBox_MigaczLewy.Checked = false; }));
                                CheckBox_MigaczPrawy.Invoke(new Action(delegate () { CheckBox_MigaczPrawy.Checked = false; }));
                                
                                StanSwiatlaAwaryjne = true;
                                EnableTimerMigacze++;
                                AllLightEnable_CheckBox();
                            }
                            if ((komenda[2] == '0') && (sendL6_flag == true))
                            {
                                StanSwiatlaAwaryjne = false;
                                pictureBox_SwiatlaAwaryjne.Image = Properties.Resources._007_hazard;
                                pictureBox_MigaczLewy.Image = Properties.Resources.arrow_50_left_gray;
                                pictureBox_MigaczPrawy.Image = Properties.Resources.arrow_50_right_gray;
                                EnableTimerMigacze--;
                                CheckBox_MigaczLewy.Invoke(new Action(delegate () { CheckBox_MigaczLewy.Checked = true; }));
                                CheckBox_MigaczPrawy.Invoke(new Action(delegate () { CheckBox_MigaczPrawy.Checked = true; }));
                                
                                AllLightEnable_CheckBox(); 
                            }
                            StartStopTimerMigacze();
                            CheckBox_SwiatlaAwaryjne.Invoke(new Action(delegate () { CheckBox_SwiatlaAwaryjne.Checked = true; }));

                            sendL6_flag = false;
                        break;

                        case '7':
                            if ((komenda[2] == '1') && (sendL7_flag == true))
                            {
                                CheckBox_SwiatlaAwaryjne.Invoke(new Action(delegate () { CheckBox_SwiatlaAwaryjne.Checked = false; }));
                                CheckBox_MigaczPrawy.Invoke(new Action(delegate () { CheckBox_MigaczPrawy.Checked = false; }));
                                StanMigaczLewy = true;
                                EnableTimerMigacze++;

                                AllLightEnable_CheckBox();
                            }
                            if ((komenda[2] == '0') && (sendL7_flag == true))
                            {
                                StanMigaczLewy = false;
                                pictureBox_MigaczLewy.Image = Properties.Resources.arrow_50_left_gray;
                                EnableTimerMigacze--;
                                CheckBox_SwiatlaAwaryjne.Invoke(new Action(delegate () { CheckBox_SwiatlaAwaryjne.Checked = true; }));
                                CheckBox_MigaczPrawy.Invoke(new Action(delegate () { CheckBox_MigaczPrawy.Checked = true; }));
                                
                                AllLightEnable_CheckBox();
                            }
                            StartStopTimerMigacze();
                            CheckBox_MigaczLewy.Invoke(new Action(delegate () { CheckBox_MigaczLewy.Checked = true; }));

                            sendL7_flag = false;
                        break;

                        case '8':
                            if ((komenda[2] == '1') && (sendL8_flag == true))
                            {
                                CheckBox_SwiatlaAwaryjne.Invoke(new Action(delegate () { CheckBox_SwiatlaAwaryjne.Checked = false; }));
                                CheckBox_MigaczLewy.Invoke(new Action(delegate () { CheckBox_MigaczLewy.Checked = false; }));
                                StanMigaczPrawy = true;
                                EnableTimerMigacze++;

                                AllLightEnable_CheckBox();
                            }
                            if ((komenda[2] == '0') && (sendL8_flag == true))
                            {
                                StanMigaczPrawy = false;
                                pictureBox_MigaczPrawy.Image = Properties.Resources.arrow_50_right_gray;
                                EnableTimerMigacze--;
                                CheckBox_SwiatlaAwaryjne.Invoke(new Action(delegate () { CheckBox_SwiatlaAwaryjne.Checked = true; }));
                                CheckBox_MigaczLewy.Invoke(new Action(delegate () { CheckBox_MigaczLewy.Checked = true; }));
                                
                                AllLightEnable_CheckBox();
                            }
                            StartStopTimerMigacze();
                            CheckBox_MigaczPrawy.Invoke(new Action(delegate () { CheckBox_MigaczPrawy.Checked = true; }));

                            sendL8_flag = false;
                        break;

                        case '9':
                            if ((komenda[2] == '1') && (sendL9_flag == true))
                            {
                                sendL9_flag = false;

                                AllLightEnable_CheckBox();
                            }
                            if ((komenda[2] == '0') && (sendL9_flag == true))
                            {
                                sendL9_flag = false;

                                AllLightEnable_CheckBox();
                            }
                        break;
                    }
                break;

                case 'P':                   //Dane pozycji
                    switch (komenda[1])
                    {
                        case 'A':                   //Dane pozycji aktualnej
                            switch (komenda[2])
                            {
                                case 'S':
                                    if(komenda[3] == '0')   //Dojechanie do pkt docelowego
                                    {
                                        Button_StopAlgorytm_Click(null, null);

                                        MessageBox.Show("Robot dojechał do celu!", "Sukces!", MessageBoxButtons.OK, MessageBoxIcon.Information);
                                    }

                                    if (komenda[3] == '1')   //Odbieram potwierdzenie
                                    {
                                        if (sendPAS_flag == true)
                                        {
                                            sendPAS_flag = false;
                                            algorytmStart = true;
                                        }
                                    }
                                    break;

                                case 'X':       //Wsp x
                                    string PAXznak = null, PAYznak = null, PAOznak = null;
                                    int PAXIndex = 0, PAYIndex = 0, PAOIndex = 0;

                                    for (int i = 0; i < komenda.Length; i++)    //Szykam gdzie sa znaczniki synchro dla wsp i orient
                                    {
                                        if (komenda[i] == '.') { komenda[i] = ','; }
                                        if (komenda[i] == 'X') { PAXIndex = i; }
                                        if (komenda[i] == 'Y') { PAYIndex = i; }
                                        if (komenda[i] == 'O') { PAOIndex = i; }
                                    }
                                    //Odczytuje wsp x
                                    for(int i = (PAXIndex + 1); i < PAYIndex; i++) { PAXznak += komenda[i]; }
                                    double.TryParse(PAXznak, out xActual);

                                    //Odczytuje wsp y
                                    for (int i = (PAYIndex + 1); i < PAOIndex; i++) { PAYznak += komenda[i]; }
                                    double.TryParse(PAYznak, out yActual);

                                    //Odczytuje orientacje
                                    for (int i = (PAOIndex + 1); i < komenda.Length; i++) { PAOznak += komenda[i]; }
                                    double fiActualKonv = 0;
                                    double.TryParse(PAOznak, out fiActualKonv);

                                    fiActual = Convert.ToInt32(fiActualKonv * (double)(180.0 / 3.1415926535));

                                    if (algorytmStart == true)
                                    {
                                        if(wybranyAlgorytm == '1')
                                        {
                                            RysowanieOsiWspolrzednych(xStart, yStart, xStop, yStop);

                                            if ((xStart != xActual) || (yStart != yActual))
                                            {
                                                ++nrWspXYActual;
                                                Array.Resize(ref tabXActual, nrWspXYActual + 1);
                                                Array.Resize(ref tabYActual, nrWspXYActual + 1);

                                                tabXActual[nrWspXYActual] = xActual;
                                                tabYActual[nrWspXYActual] = yActual;

                                                RysowanieSciezkiRobota(tabXActual, tabYActual);

                                                RysowanieUkladuWspolrzednych(xActual, yActual, fiActual, szerokoscKratkix, szerokoscKratkiy, "orange");
                                            }

                                            RysowanieUkladuWspolrzednych(xStart, yStart, fiStart, szerokoscKratkix, szerokoscKratkiy, "green");
                                            RysowanieUkladuWspolrzednych(xStop, yStop, fiStop, szerokoscKratkix, szerokoscKratkiy, "red");
                                        }

                                        if ((wybranyAlgorytm == '2') || (wybranyAlgorytm == '3'))
                                        {
                                            if ((xStart == xActual) && (yStart == yActual))
                                            {
                                                RysowanieOsiWspolrzednych(xStart, yStart, 0.0, 0.0);
                                            }
                                            else
                                            {
                                                if (xActual > xMaxValue) { xMaxValue = xActual; }   //Wyznaczene x min i max tam gdzie robot jezdzil
                                                if (xActual < xMinValue) { xMinValue = xActual; }
                                                if (yActual > yMaxValue) { yMaxValue = yActual; }   //Wyznaczene y min i max tam gdzie robot jezdzil
                                                if (yActual < yMinValue) { yMinValue = yActual; }

                                                RysowanieOsiWspolrzednych(xMinValue, yMinValue, xMaxValue, yMaxValue);

                                                ++nrWspXYActual;
                                                tb2.Text = Convert.ToString(nrWspXYActual);
                                                Array.Resize(ref tabXActual, nrWspXYActual + 1);
                                                Array.Resize(ref tabYActual, nrWspXYActual + 1);

                                                tabXActual[nrWspXYActual] = xActual;
                                                tabYActual[nrWspXYActual] = yActual;

                                                RysowanieSciezkiRobota(tabXActual, tabYActual);

                                                RysowanieUkladuWspolrzednych(xActual, yActual, fiActual, szerokoscKratkix, szerokoscKratkiy, "orange");
                                            }
                                            
                                            RysowanieUkladuWspolrzednych(xStart, yStart, fiStart, szerokoscKratkix, szerokoscKratkiy, "green");
                                        }
                                    }
                                break;
                            }
                        break;

                        case 'G':
                            switch (komenda[2])
                            {
                                case 'X':       //Wsp x
                                    if (sendPGX_flag == true)
                                    {
                                        try
                                        {
                                            string xCheckStopWsp = "PGX" + komenda[3] + komenda[4] + komenda[5] + komenda[6] + komenda[7];

                                            if (xCheckStopWsp == xStopWsp) //Jesli komenda zostala dobrze odebrana
                                            {
                                                sendPGX_flag = false;

                                                formSendData.label_StatusXStop.ForeColor = Color.Green;
                                                formSendData.label_StatusXStop.Text = "Wysłano";

                                                SetValueOnTrackbar();
                                            }
                                            else
                                            {
                                                formSendData.label_StatusXStop.ForeColor = Color.Red;
                                                formSendData.label_StatusXStop.Text = "Ponawianie";
                                                sendDataToMobileRobot(xStopWsp, 'y');
                                            }
                                        }
                                        catch
                                        {
                                            formSendData.label_StatusXStop.ForeColor = Color.Red;
                                            formSendData.label_StatusXStop.Text = "Ponawianie";
                                            sendDataToMobileRobot(xStopWsp, 'y');
                                        }
                                    }
                                break;

                                case 'Y':       //Wsp y
                                    if (sendPGY_flag == true)
                                    {
                                        try
                                        {
                                            string yCheckStopWsp = "PGY" + komenda[3] + komenda[4] + komenda[5] + komenda[6] + komenda[7];

                                            if (yCheckStopWsp == yStopWsp) //Jesli komenda zostala dobrze odebrana
                                            {
                                                sendPGY_flag = false;

                                                formSendData.label_StatusYStop.ForeColor = Color.Green;
                                                formSendData.label_StatusYStop.Text = "Wysłano";

                                                SetValueOnTrackbar();
                                            }
                                            else
                                            {
                                                formSendData.label_StatusYStop.ForeColor = Color.Red;
                                                formSendData.label_StatusYStop.Text = "Ponawianie";
                                                sendDataToMobileRobot(yStopWsp, 'y');
                                            }
                                        }
                                        catch
                                        {
                                            formSendData.label_StatusYStop.ForeColor = Color.Red;
                                            formSendData.label_StatusYStop.Text = "Ponawianie";
                                            sendDataToMobileRobot(yStopWsp, 'y');
                                        }
                                    }
                                break;

                                case 'O':       //Orientacja
                                    if (sendPGO_flag == true)
                                    {
                                        try
                                        {
                                            string fiCheckStopWsp = "PGO" + komenda[3] + komenda[4] + komenda[5];

                                            if (fiCheckStopWsp == fiStopWsp) //Jesli komenda zostala dobrze odebrana
                                            {
                                                sendPGO_flag = false;

                                                formSendData.label_StatusFiStop.ForeColor = Color.Green;
                                                formSendData.label_StatusFiStop.Text = "Wysłano";

                                                SetValueOnTrackbar();
                                            }
                                            else
                                            {
                                                formSendData.label_StatusFiStop.ForeColor = Color.Red;
                                                formSendData.label_StatusFiStop.Text = "Ponawianie";
                                                sendDataToMobileRobot(fiStopWsp, 'y');
                                            }
                                        }
                                        catch
                                        {
                                            formSendData.label_StatusFiStop.ForeColor = Color.Red;
                                            formSendData.label_StatusFiStop.Text = "Ponawianie";
                                            sendDataToMobileRobot(fiStopWsp, 'y');
                                        }
                                    }
                                break;

                                case 'S':
                                    switch (komenda[3])
                                    {
                                        case 'A':
                                            if (sendGSA_flag == true)
                                            {
                                                try
                                                {
                                                    string CheckGSA = "PGSA" + komenda[4] + komenda[5] + komenda[6];

                                                    if (CheckGSA == stringGSA) //Jesli komenda zostala dobrze odebrana
                                                    {
                                                        sendGSA_flag = false;

                                                        formSendData.label_StatusGA.ForeColor = Color.Green;
                                                        formSendData.label_StatusGA.Text = "Wysłano";

                                                        SetValueOnTrackbar();
                                                    }
                                                    else
                                                    {
                                                        formSendData.label_StatusGA.ForeColor = Color.Red;
                                                        formSendData.label_StatusGA.Text = "Ponawianie";
                                                        sendDataToMobileRobot(stringGSA, 'y');
                                                    }
                                                }
                                                catch
                                                {
                                                    formSendData.label_StatusGA.ForeColor = Color.Red;
                                                    formSendData.label_StatusGA.Text = "Ponawianie";
                                                    sendDataToMobileRobot(stringGSA, 'y');
                                                }
                                            }
                                        break;

                                        case 'W':
                                            if (sendGSW_flag == true)
                                            {
                                                try
                                                {
                                                    string CheckGSW = "PGSW" + komenda[4] + komenda[5] + komenda[6];

                                                    if (CheckGSW == stringGW) //Jesli komenda zostala dobrze odebrana
                                                    {
                                                        sendGSW_flag = false;

                                                        formSendData.label_StatusGW.ForeColor = Color.Green;
                                                        formSendData.label_StatusGW.Text = "Wysłano";

                                                        SetValueOnTrackbar();
                                                    }
                                                    else
                                                    {
                                                        formSendData.label_StatusGW.ForeColor = Color.Red;
                                                        formSendData.label_StatusGW.Text = "Ponawianie";
                                                        sendDataToMobileRobot(stringGW, 'y');
                                                    }
                                                }
                                                catch
                                                {
                                                    formSendData.label_StatusGW.ForeColor = Color.Red;
                                                    formSendData.label_StatusGW.Text = "Ponawianie";
                                                    sendDataToMobileRobot(stringGW, 'y');
                                                }
                                            }
                                        break;

                                        case 'G':
                                            if (sendGSG_flag == true)
                                            {
                                                try
                                                {
                                                    string CheckGSG = "PGSG" + komenda[4] + komenda[5] + komenda[6];

                                                    if (CheckGSG == stringGG) //Jesli komenda zostala dobrze odebrana
                                                    {
                                                        sendGSG_flag = false;

                                                        formSendData.label_StatusGG.ForeColor = Color.Green;
                                                        formSendData.label_StatusGG.Text = "Wysłano";

                                                        SetValueOnTrackbar();
                                                    }
                                                    else
                                                    {
                                                        formSendData.label_StatusGG.ForeColor = Color.Red;
                                                        formSendData.label_StatusGG.Text = "Ponawianie";
                                                        sendDataToMobileRobot(stringGG, 'y');
                                                    }
                                                }
                                                catch
                                                {
                                                    formSendData.label_StatusGG.ForeColor = Color.Red;
                                                    formSendData.label_StatusGG.Text = "Ponawianie";
                                                    sendDataToMobileRobot(stringGG, 'y');
                                                }
                                            }
                                        break;
                                    }
                                break;
                            }
                        break;

                        case 'M':
                            switch (komenda[2])
                            {
                                case 'V':
                                    if (sendMSFMR_flag == true)
                                    {
                                        try
                                        {
                                            string CheckMSFMR = "PMV" + komenda[3] + komenda[4] + komenda[5];

                                            if (CheckMSFMR == stringMaxSFMR) //Jesli komenda zostala dobrze odebrana
                                            {
                                                sendMSFMR_flag = false;

                                                if (wybranyAlgorytm == '2')
                                                {
                                                    formSendData.label_StatusSubMaxV.ForeColor = Color.Green;
                                                    formSendData.label_StatusSubMaxV.Text = "Wysłano";
                                                }
                                                if (wybranyAlgorytm == '3')
                                                {
                                                    formSendData.label_StatusMotMaxV.ForeColor = Color.Green;
                                                    formSendData.label_StatusMotMaxV.Text = "Wysłano";
                                                }

                                                SetValueOnTrackbar();
                                            }
                                            else
                                            {
                                                if (wybranyAlgorytm == '2')
                                                {
                                                    formSendData.label_StatusSubMaxV.ForeColor = Color.Red;
                                                    formSendData.label_StatusSubMaxV.Text = "Ponawianie";
                                                    sendDataToMobileRobot(stringMaxSFMR, 'y');
                                                }
                                                if (wybranyAlgorytm == '3')
                                                {
                                                    formSendData.label_StatusMotMaxV.ForeColor = Color.Red;
                                                    formSendData.label_StatusMotMaxV.Text = "Ponawianie";
                                                    sendDataToMobileRobot(stringMaxSFMR, 'y');
                                                }
                                            }
                                        }
                                        catch
                                        {
                                            if (wybranyAlgorytm == '2')
                                            {
                                                formSendData.label_StatusSubMaxV.ForeColor = Color.Red;
                                                formSendData.label_StatusSubMaxV.Text = "Ponawianie";
                                                sendDataToMobileRobot(stringMaxSFMR, 'y');
                                            }
                                            if (wybranyAlgorytm == '3')
                                            {
                                                formSendData.label_StatusMotMaxV.ForeColor = Color.Red;
                                                formSendData.label_StatusMotMaxV.Text = "Ponawianie";
                                                sendDataToMobileRobot(stringMaxSFMR, 'y');
                                            }
                                        }
                                    }
                                    break;
                            }
                        break;

                        case 'S':
                            switch (komenda[2])
                            {
                                case 'A':
                                    switch (komenda[3])
                                    {
                                        case '1':
                                            if (sendDSA1_flag == true)
                                            {
                                                try
                                                {
                                                    string CheckDSA1 = "PSA1" + komenda[4] + komenda[5];

                                                    if (CheckDSA1 == stringDSA1) //Jesli komenda zostala dobrze odebrana
                                                    {
                                                        sendDSA1_flag = false;

                                                        formSendData.label_StatusDSA1.ForeColor = Color.Green;
                                                        formSendData.label_StatusDSA1.Text = "Wysłano";

                                                        SetValueOnTrackbar();
                                                    }
                                                    else
                                                    {
                                                        formSendData.label_StatusDSA1.ForeColor = Color.Red;
                                                        formSendData.label_StatusDSA1.Text = "Ponawianie";
                                                        sendDataToMobileRobot(stringDSA1, 'y');
                                                    }
                                                }
                                                catch
                                                {
                                                    formSendData.label_StatusDSA1.ForeColor = Color.Red;
                                                    formSendData.label_StatusDSA1.Text = "Ponawianie";
                                                    sendDataToMobileRobot(stringDSA1, 'y');
                                                }
                                            }
                                        break;

                                        case '2':
                                            if (sendDSA2_flag == true)
                                            {
                                                try
                                                {
                                                    string CheckDSA2 = "PSA2" + komenda[4] + komenda[5];

                                                    if (CheckDSA2 == stringDSA2) //Jesli komenda zostala dobrze odebrana
                                                    {
                                                        sendDSA2_flag = false;

                                                        formSendData.label_StatusDSA2.ForeColor = Color.Green;
                                                        formSendData.label_StatusDSA2.Text = "Wysłano";

                                                        SetValueOnTrackbar();
                                                    }
                                                    else
                                                    {
                                                        formSendData.label_StatusDSA2.ForeColor = Color.Red;
                                                        formSendData.label_StatusDSA2.Text = "Ponawianie";
                                                        sendDataToMobileRobot(stringDSA2, 'y');
                                                    }
                                                }
                                                catch
                                                {
                                                    formSendData.label_StatusDSA2.ForeColor = Color.Red;
                                                    formSendData.label_StatusDSA2.Text = "Ponawianie";
                                                    sendDataToMobileRobot(stringDSA2, 'y');
                                                }
                                            }
                                        break;

                                        case '3':
                                            if (sendDSA3_flag == true)
                                            {
                                                try
                                                {
                                                    string CheckDSA3 = "PSA3" + komenda[4] + komenda[5];

                                                    if (CheckDSA3 == stringDSA3) //Jesli komenda zostala dobrze odebrana
                                                    {
                                                        sendDSA3_flag = false;

                                                        formSendData.label_StatusDSA3.ForeColor = Color.Green;
                                                        formSendData.label_StatusDSA3.Text = "Wysłano";

                                                        SetValueOnTrackbar();
                                                    }
                                                    else
                                                    {
                                                        formSendData.label_StatusDSA3.ForeColor = Color.Red;
                                                        formSendData.label_StatusDSA3.Text = "Ponawianie";
                                                        sendDataToMobileRobot(stringDSA3, 'y');
                                                    }
                                                }
                                                catch
                                                {
                                                    formSendData.label_StatusDSA3.ForeColor = Color.Red;
                                                    formSendData.label_StatusDSA3.Text = "Ponawianie";
                                                    sendDataToMobileRobot(stringDSA3, 'y');
                                                }
                                            }
                                        break;

                                        case '4':
                                            if (sendDSA4_flag == true)
                                            {
                                                try
                                                {
                                                    string CheckDSA4 = "PSA4" + komenda[4] + komenda[5];

                                                    if (CheckDSA4 == stringDSA4) //Jesli komenda zostala dobrze odebrana
                                                    {
                                                        sendDSA4_flag = false;

                                                        formSendData.label_StatusDSA4.ForeColor = Color.Green;
                                                        formSendData.label_StatusDSA4.Text = "Wysłano";

                                                        SetValueOnTrackbar();
                                                    }
                                                    else
                                                    {
                                                        formSendData.label_StatusDSA4.ForeColor = Color.Red;
                                                        formSendData.label_StatusDSA4.Text = "Ponawianie";
                                                        sendDataToMobileRobot(stringDSA4, 'y');
                                                    }
                                                }
                                                catch
                                                {
                                                    formSendData.label_StatusDSA4.ForeColor = Color.Red;
                                                    formSendData.label_StatusDSA4.Text = "Ponawianie";
                                                    sendDataToMobileRobot(stringDSA4, 'y');
                                                }
                                            }
                                        break;

                                        case '5':
                                            if (sendDSA5_flag == true)
                                            {
                                                try
                                                {
                                                    string CheckDSA5 = "PSA5" + komenda[4] + komenda[5];

                                                    if (CheckDSA5 == stringDSA5) //Jesli komenda zostala dobrze odebrana
                                                    {
                                                        sendDSA5_flag = false;

                                                        formSendData.label_StatusDSA5.ForeColor = Color.Green;
                                                        formSendData.label_StatusDSA5.Text = "Wysłano";

                                                        SetValueOnTrackbar();
                                                    }
                                                    else
                                                    {
                                                        formSendData.label_StatusDSA5.ForeColor = Color.Red;
                                                        formSendData.label_StatusDSA5.Text = "Ponawianie";
                                                        sendDataToMobileRobot(stringDSA5, 'y');
                                                    }
                                                }
                                                catch
                                                {
                                                    formSendData.label_StatusDSA5.ForeColor = Color.Red;
                                                    formSendData.label_StatusDSA5.Text = "Ponawianie";
                                                    sendDataToMobileRobot(stringDSA5, 'y');
                                                }
                                            }
                                        break;
                                    }
                                break;

                                case 'X':       //Wsp x
                                    if (sendPSX_flag == true)
                                    {
                                        try
                                        {
                                            string xCheckStartWsp = "PSX" + komenda[3] + komenda[4] + komenda[5] + komenda[6] + komenda[7];

                                            if (xCheckStartWsp == xStartWsp) //Jesli komenda zostala dobrze odebrana
                                            {
                                                sendPSX_flag = false;

                                                formSendData.label_StatusXStart.ForeColor = Color.Green;
                                                formSendData.label_StatusXStart.Text = "Wysłano";

                                                SetValueOnTrackbar();
                                            }
                                            else
                                            {
                                                formSendData.label_StatusXStart.ForeColor = Color.Red;
                                                formSendData.label_StatusXStart.Text = "Ponawianie";
                                                sendDataToMobileRobot(xStartWsp, 'y');
                                            }
                                        }
                                        catch
                                        {
                                            formSendData.label_StatusXStart.ForeColor = Color.Red;
                                            formSendData.label_StatusXStart.Text = "Ponawianie";
                                            sendDataToMobileRobot(xStartWsp, 'y');
                                        }
                                    }
                                break;

                                case 'Y':       //Wsp y
                                    if (sendPSY_flag == true)
                                    {
                                        try
                                        {
                                            string yCheckStartWsp = "PSY" + komenda[3] + komenda[4] + komenda[5] + komenda[6] + komenda[7];

                                            if (yCheckStartWsp == yStartWsp) //Jesli komenda zostala dobrze odebrana
                                            {
                                                sendPSY_flag = false;

                                                formSendData.label_StatusYStart.ForeColor = Color.Green;
                                                formSendData.label_StatusYStart.Text = "Wysłano";

                                                SetValueOnTrackbar();
                                            }
                                            else
                                            {
                                                formSendData.label_StatusYStart.ForeColor = Color.Red;
                                                formSendData.label_StatusYStart.Text = "Ponawianie";
                                                sendDataToMobileRobot(yStartWsp, 'y');
                                            }
                                        }
                                        catch
                                        {
                                            formSendData.label_StatusYStart.ForeColor = Color.Red;
                                            formSendData.label_StatusYStart.Text = "Ponawianie";
                                            sendDataToMobileRobot(yStartWsp, 'y');
                                        }
                                    }
                                break;

                                case 'O':       //Orientacja
                                    if (sendPSO_flag == true)
                                    {
                                        try
                                        {
                                            string fiCheckStartWsp = "PSO" + komenda[3] + komenda[4] + komenda[5];

                                            if (fiCheckStartWsp == fiStartWsp) //Jesli komenda zostala dobrze odebrana
                                            {
                                                sendPSO_flag = false;

                                                formSendData.label_StatusFiStart.ForeColor = Color.Green;
                                                formSendData.label_StatusFiStart.Text = "Wysłano";

                                                SetValueOnTrackbar();
                                            }
                                            else
                                            {
                                                formSendData.label_StatusFiStart.ForeColor = Color.Red;
                                                formSendData.label_StatusFiStart.Text = "Ponawianie";
                                                sendDataToMobileRobot(fiStartWsp, 'y');
                                            }
                                        }
                                        catch
                                        {
                                            formSendData.label_StatusFiStart.ForeColor = Color.Red;
                                            formSendData.label_StatusFiStart.Text = "Ponawianie";
                                            sendDataToMobileRobot(fiStartWsp, 'y');
                                        }
                                    }
                                break;
                            }
                        break;

                        case 'W':
                            switch (komenda[2])
                            {
                                case 'A':
                                    if (sendPWA_flag == true)
                                    {
                                        try
                                        {
                                            string CheckWybranyAlgorytm = "PWA" + komenda[3];

                                            if (CheckWybranyAlgorytm == checkAlgorytm) //Jesli komenda zostala dobrze odebrana
                                            {
                                                sendPWA_flag = false;

                                                formSendData.label_StatusWybranyAlgorytm.ForeColor = Color.Green;
                                                formSendData.label_StatusWybranyAlgorytm.Text = "Wysłano";

                                                SetValueOnTrackbar();
                                            }
                                            else
                                            {
                                                formSendData.label_StatusWybranyAlgorytm.ForeColor = Color.Red;
                                                formSendData.label_StatusWybranyAlgorytm.Text = "Ponawianie";
                                                sendDataToMobileRobot(checkAlgorytm, 'y');
                                            }
                                        }
                                        catch
                                        {
                                            formSendData.label_StatusWybranyAlgorytm.ForeColor = Color.Red;
                                            formSendData.label_StatusWybranyAlgorytm.Text = "Ponawianie";
                                            sendDataToMobileRobot(checkAlgorytm, 'y');
                                        }
                                    }
                                    break;
                            }
                        break;
                    }
                break;

                case 'R':                   //Dane wysylane z robota
                    switch (komenda[1])
                    {
                        case 'D':
                            switch (komenda[2])
                            {
                                case 'G':
                                    string CMUCam_RozmiarObietu = null;
                                    try
                                    {
                                        for (int i = 3; i < komenda.Length; i++)
                                        {
                                            CMUCam_RozmiarObietu += komenda[i];
                                        }
                                        listBox_WypiszDaneOdebrneWyslane(CMUCam_RozmiarObietu, 'r', 'n');
                                        textBox_PoleObiektu.Text = CMUCam_RozmiarObietu;
                                    }
                                    catch
                                    {

                                    }
                                break;
                            }
                        break;

                        case 'M':
                            switch (komenda[2])
                            {
                                case 'S':   //Dane o wektorach strowan (Motor Schemas)
                                    string MSM1, MSM2, MSM3, MSM;    //Motor Schemas Magnitude
                                    string MSD1, MSD2, MSD3, MSD;    //Motor Schemas Direction

                                    try
                                    {
                                        for (int i = 0; i < 8; i++)  //odczytuje 5 wektorow wyslanych z robota
                                        {
                                            MSM1 = Convert.ToString(komenda[(i * 6) + 4]);
                                            MSM2 = Convert.ToString(komenda[(i * 6) + 5]);
                                            MSM3 = Convert.ToString(komenda[(i * 6) + 6]);
                                            MSM = MSM1 + MSM2 + MSM3;
                                            int.TryParse(MSM, out vectorMagnitude[i]);

                                            MSD1 = Convert.ToString(komenda[(i * 6) + 7]);
                                            MSD2 = Convert.ToString(komenda[(i * 6) + 8]);
                                            MSD3 = Convert.ToString(komenda[(i * 6) + 9]);
                                            MSD = MSD1 + MSD2 + MSD3;
                                            int.TryParse(MSD, out vectorDirection[i]);
                                        }
                                        rodzajZachowaniaMS = komenda[3];
                                        RysowanieWektorowStrowan(vectorMagnitude, vectorDirection, komenda[3]);
                                    }
                                    catch
                                    {

                                    }
                                break;
                            }
                        break;

                        case 'S':
                            switch (komenda[2])
                            {
                                case 'F':   //Dane z przednich czujnikow
                                    DaneOdleglosciPrzod[0] = 0; //Okreslam ze to dane dla czujnikow przednich

                                    try
                                    {
                                        for (int i = 1; i <= 6; i++)     //Odczytuje wszystkie dane pokolei
                                        {
                                            string sensorF1 = Convert.ToString(komenda[(i * 2 - 1) + 2]);   //liczba dziesiatek
                                            string sensorF2 = Convert.ToString(komenda[(i * 2) + 2]);       //liczba jednosci
                                            string sensorF = sensorF1 + sensorF2;

                                            int.TryParse(sensorF, out DaneOdleglosciPrzod[i]);
                                        }

                                        RysowanieSiatkiRadaru();
                                    }
                                    catch
                                    {

                                    }
                                break;

                                case 'P':   //Predkosc robota
                                    try
                                    {
                                        string Sznak1 = Convert.ToString(komenda[3]);
                                        string Sznak2 = Convert.ToString(komenda[5]);
                                        string Sznak3 = Convert.ToString(komenda[6]);
                                        string Sznak = Sznak1 + Sznak2 + Sznak3;

                                        float.TryParse(Sznak, out SpeedMobileRobot);

                                        GaugeSpeedometer.Value = SpeedMobileRobot;
                                        textBox3_Speed.Text = Convert.ToString(SpeedMobileRobot);

                                        if (algorytmStart == true)   //Aktualizacja tablicy predkosci tylko jesli robot wykonuje algorytm
                                        {
                                            for (int i = (tabSpeedMobileRobot.Length - 1); i > 0; i--)   //Aktualizacja tablicy predkosci
                                            {
                                                tabSpeedMobileRobot[i] = tabSpeedMobileRobot[i - 1];
                                            }
                                            tabSpeedMobileRobot[0] = SpeedMobileRobot;

                                            RysowanieOsiPredkosci();
                                        }
                                    }
                                    catch
                                    {

                                    }
                                break;
                                    
                                case 'R':   //Dane z tylnich czujnikow
                                    DaneOdleglosciTyl[0] = 1; //Okreslam ze to dane dla czujnikow przednich

                                    try
                                    {
                                        for (int i = 1; i <= 6; i++)     //Odczytuje wszystkie dane pokolei
                                        {
                                            string sensorR1 = Convert.ToString(komenda[(i * 2 - 1) + 2]);   //liczba dziesiatek
                                            string sensorR2 = Convert.ToString(komenda[(i * 2) + 2]);       //liczba jednosci
                                            string sensorR = sensorR1 + sensorR2;

                                            int.TryParse(sensorR, out DaneOdleglosciTyl[i]);
                                        }

                                        RysowanieSiatkiRadaru();
                                    }
                                    catch
                                    {

                                    }
                                break;

                                case 'S':   //Dane o aktualnym stanie automatu (Subsumption)
                                    stanAutomatuWanderGoal = komenda[3];
                                    RysowanieAutomatuStanu(komenda[3]);
                                break;
                            }
                            
                        break;

                        case 'T':
                            switch (komenda[2])
                            {
                                case 'E':
                                    try
                                    {
                                        string Tznak1 = Convert.ToString(komenda[3]);
                                        string Tznak2 = Convert.ToString(komenda[4]);
                                        string Tznak3 = Convert.ToString(komenda[6]);
                                        string Tznak = Tznak1 + Tznak2 + Tznak3;

                                        float.TryParse(Tznak, out TempCPUMobileRobot);

                                        TempCPUMobileRobot = TempCPUMobileRobot / 10;
                                        GaugeTempCPU.Value = TempCPUMobileRobot;

                                        Tznak = Convert.ToString(TempCPUMobileRobot);
                                        Tznak += " 'C";

                                        textBox_TempCPU.Text = Tznak;
                                    }
                                    catch
                                    {

                                    }
                                break;
                            }
                            
                        break;
                    }
                break;
            }
        }

        void SetValueOnTrackbar()
        {
            if (wybranyAlgorytm == '1') formSendData.progressBar_SendDataToMobileRobot.Maximum = 7;
            if (wybranyAlgorytm == '2') formSendData.progressBar_SendDataToMobileRobot.Maximum = 10;
            if (wybranyAlgorytm == '3') formSendData.progressBar_SendDataToMobileRobot.Maximum = 8;

            ++progressBarSendDataToMobileRobot;

            formSendData.progressBar_SendDataToMobileRobot.Value = progressBarSendDataToMobileRobot;

            if ((wybranyAlgorytm == '1' && progressBarSendDataToMobileRobot == 7) || (wybranyAlgorytm == '2' && progressBarSendDataToMobileRobot == 10) || (wybranyAlgorytm == '3' && progressBarSendDataToMobileRobot == 8))
            {
                formSendData.Button_OK.Enabled = true;
            }
        }

        private void Button_ControlFromKeyboard_Click(object sender, EventArgs e)
        {
            //Włączenie timera i sprzwdzenie czy wciskany jest przycisk
        }

        private void Button_GearManual_Click(object sender, EventArgs e)    //Przelaczenie na skrzynie manualna
        {
             //Zablokowanie przycisku na czas wyslania rozkazu i otrzymania potwierdzenia
            Button_GearManual.BackColor = Color.Transparent;
            Button_GearManual.Enabled = false;
            Button_GearAutomatic.Enabled = false;

            sendDataToMobileRobot("GM", 'n');
            sendGM_flag = true;
            timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
        }
        private void Button_GearAutomatic_Click(object sender, EventArgs e)
        {
            Button_GearManual.Enabled = false;
            Button_GearAutomatic.BackColor = Color.Transparent;
            Button_GearAutomatic.Enabled = false;
            AllGearDisable_Button();

            sendDataToMobileRobot("GA", 'n');
            sendGA_flag = true;
            timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
        }
        private void Button_GearN_Click(object sender, EventArgs e)
        {
            Button_GearManual.Enabled = false;
            Button_GearAutomatic.Enabled = false;
            AllGearDisable_Button();

            sendDataToMobileRobot("GN", 'n');
            sendN_flag = true;
            timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
        }
        private void Button_Gear1_Click(object sender, EventArgs e)
        {
            Button_GearManual.Enabled = false;
            Button_GearAutomatic.Enabled = false;
            AllGearDisable_Button();

            sendDataToMobileRobot("G1", 'n');
            send1_flag = true;
            timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
        }
        private void Button_Gear2_Click(object sender, EventArgs e)
        {
            Button_GearManual.Enabled = false;
            Button_GearAutomatic.Enabled = false;
            AllGearDisable_Button();

            sendDataToMobileRobot("G2", 'n');
            send2_flag = true;
            timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
        }
        private void Button_Gear3_Click(object sender, EventArgs e)
        {
            Button_GearManual.Enabled = false;
            Button_GearAutomatic.Enabled = false;
            AllGearDisable_Button();

            sendDataToMobileRobot("G3", 'n');
            send3_flag = true;
            timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
        }
        private void Button_Gear4_Click(object sender, EventArgs e)
        {
            Button_GearManual.Enabled = false;
            Button_GearAutomatic.Enabled = false;
            AllGearDisable_Button();

            sendDataToMobileRobot("G4", 'n');
            send4_flag = true;
            timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
        }
        private void Button_Gear5_Click(object sender, EventArgs e)
        {
            Button_GearManual.Enabled = false;
            Button_GearAutomatic.Enabled = false;
            AllGearDisable_Button();

            sendDataToMobileRobot("G5", 'n');
            send5_flag = true;
            timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
        }
        private void Button_Gear6_Click(object sender, EventArgs e)
        {
            Button_GearManual.Enabled = false;
            Button_GearAutomatic.Enabled = false;
            AllGearDisable_Button();

            sendDataToMobileRobot("G6", 'n');
            send6_flag = true;
            timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
        }
        private void Button_GearR_Click(object sender, EventArgs e)
        {
            Button_GearManual.Enabled = false;
            Button_GearAutomatic.Enabled = false;
            AllGearDisable_Button();

            sendDataToMobileRobot("GR", 'n');
            sendR_flag = true;
            timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
        }

        void sendDataToMobileRobot(string text, char repeat)
        {
            try
            {
                sp.WriteLine(text);
                listBox_WypiszDaneOdebrneWyslane(text, 'w', repeat);
            }
            catch
            {
                listBox_WypiszDaneOdebrneWyslane(text, 'w', 'e');
            }
        }

        void AllLightEnable_CheckBox()
        {
            CheckBox_SwiatlaDzienne.Invoke(new Action(delegate ()               { CheckBox_SwiatlaDzienne.Enabled               = true; }));
            CheckBox_SwiatlaPostojowe.Invoke(new Action(delegate ()             { CheckBox_SwiatlaPostojowe.Enabled             = true; }));
            CheckBox_SwiatlaKrotkie.Invoke(new Action(delegate ()               { CheckBox_SwiatlaKrotkie.Enabled               = true; }));
            CheckBox_SwiatlaDlugie.Invoke(new Action(delegate ()                { CheckBox_SwiatlaDlugie.Enabled                = true; }));
            CheckBox_SwiatlaPrzeciwmglowePrzednie.Invoke(new Action(delegate () { CheckBox_SwiatlaPrzeciwmglowePrzednie.Enabled = true; }));
            CheckBox_SwiatlaPrzeciwmgloweTylne.Invoke(new Action(delegate ()    { CheckBox_SwiatlaPrzeciwmgloweTylne.Enabled    = true; }));
            CheckBox_SwiatlaAwaryjne.Invoke(new Action(delegate ()              { CheckBox_SwiatlaAwaryjne.Enabled              = true; }));
            CheckBox_MigaczLewy.Invoke(new Action(delegate ()                   { CheckBox_MigaczLewy.Enabled                   = true; }));
            CheckBox_MigaczPrawy.Invoke(new Action(delegate ()                  { CheckBox_MigaczPrawy.Enabled                  = true; }));
            CheckBox_Klakson.Invoke(new Action(delegate ()                      { CheckBox_Klakson.Enabled                      = true; }));
        }
        void AllLightDisable_CheckBox()
        {
            CheckBox_SwiatlaDzienne.Invoke(new Action(delegate ()               { CheckBox_SwiatlaDzienne.Enabled               = false; }));
            CheckBox_SwiatlaPostojowe.Invoke(new Action(delegate ()             { CheckBox_SwiatlaPostojowe.Enabled             = false; }));
            CheckBox_SwiatlaKrotkie.Invoke(new Action(delegate ()               { CheckBox_SwiatlaKrotkie.Enabled               = false; }));
            CheckBox_SwiatlaDlugie.Invoke(new Action(delegate ()                { CheckBox_SwiatlaDlugie.Enabled                = false; }));
            CheckBox_SwiatlaPrzeciwmglowePrzednie.Invoke(new Action(delegate () { CheckBox_SwiatlaPrzeciwmglowePrzednie.Enabled = false; }));
            CheckBox_SwiatlaPrzeciwmgloweTylne.Invoke(new Action(delegate ()    { CheckBox_SwiatlaPrzeciwmgloweTylne.Enabled    = false; }));
            CheckBox_SwiatlaAwaryjne.Invoke(new Action(delegate ()              { CheckBox_SwiatlaAwaryjne.Enabled              = false; }));
            CheckBox_MigaczLewy.Invoke(new Action(delegate ()                   { CheckBox_MigaczLewy.Enabled                   = false; }));
            CheckBox_MigaczPrawy.Invoke(new Action(delegate ()                  { CheckBox_MigaczPrawy.Enabled                  = false; }));
            CheckBox_Klakson.Invoke(new Action(delegate ()                      { CheckBox_Klakson.Enabled                      = false; }));
        }

        private void CheckBox_SwiatlaDzienne_CheckedChanged(object sender, EventArgs e)
        {
            if (sendL0_flag == false)
            {
                AllLightDisable_CheckBox();

                sendDataToMobileRobot("L0", 'n');
                sendL0_flag = true;
                timer_SprawdzKomendeUART.Stop();    timer_SprawdzKomendeUART.Start();
            }
        }
        private void CheckBox_SwiatlaPostojowe_CheckedChanged(object sender, EventArgs e)
        {
            if (sendL1_flag == false)
            {
                AllLightDisable_CheckBox();

                sendDataToMobileRobot("L1", 'n');
                sendL1_flag = true;
                timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
            }
        }
        private void CheckBox_SwiatlaKrotkie_CheckedChanged(object sender, EventArgs e)
        {
            if (sendL2_flag == false)
            {
                AllLightDisable_CheckBox();

                sendDataToMobileRobot("L2", 'n');
                sendL2_flag = true;
                timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
            }
        }
        private void CheckBox_SwiatlaDlugie_CheckedChanged(object sender, EventArgs e)
        {
            if (sendL3_flag == false)
            {
                AllLightDisable_CheckBox();

                sendDataToMobileRobot("L3", 'n');
                sendL3_flag = true;
                timer_SprawdzKomendeUART.Stop();    timer_SprawdzKomendeUART.Start();
            }
        }
        private void CheckBox_SwiatlaPrzeciwmglowePrzednie_CheckedChanged(object sender, EventArgs e)
        {
            /*CheckBox_SwiatlaPrzeciwmglowePrzednie.Enabled = false;
            //wysylam komende zapalenia
            //czekam na potwierdzenie wlaczenia
            //if(wlaczylo sie)

            if (StanSwiatlaPrzeciwmglowePrzednie == false)
            {
                pictureBox_SwiatlaPrzeciwmglowePrzednie.Image = Properties.Resources._005_fog_light_green;
                StanSwiatlaPrzeciwmglowePrzednie = true;
                CheckBox_SwiatlaPrzeciwmglowePrzednie.Enabled = true;
            }
            else if (StanSwiatlaPrzeciwmglowePrzednie == true)
            {
                pictureBox_SwiatlaPrzeciwmglowePrzednie.Image = Properties.Resources._005_fog_light;
                StanSwiatlaPrzeciwmglowePrzednie = false;
                CheckBox_SwiatlaPrzeciwmglowePrzednie.Enabled = true;
            }*/
        }
        private void CheckBox_SwiatlaPrzeciwmgloweTylne_CheckedChanged(object sender, EventArgs e)
        {
            if (sendL5_flag == false)
            {
                AllLightDisable_CheckBox();

                sendDataToMobileRobot("L5", 'n');
                sendL5_flag = true;
                timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
            }
        }
        private void CheckBox_SwiatlaAwaryjne_CheckedChanged(object sender, EventArgs e)
        {
            if (sendL6_flag == false)
            {
                AllLightDisable_CheckBox();

                sendDataToMobileRobot("L6", 'n');
                sendL6_flag = true;
                timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
            }
        }
        private void CheckBox_MigaczLewy_CheckedChanged(object sender, EventArgs e)
        {
            if (sendL7_flag == false)
            {
                AllLightDisable_CheckBox();

                sendDataToMobileRobot("L7", 'n');
                sendL7_flag = true;
                timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
            }
        }
        private void CheckBox_MigaczPrawy_CheckedChanged(object sender, EventArgs e)
        {
            if (sendL8_flag == false)
            {
                AllLightDisable_CheckBox();

                sendDataToMobileRobot("L8", 'n');
                sendL8_flag = true;
                timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
            }
        }
        private void CheckBox_Klakson_CheckedChanged(object sender, EventArgs e)
        {
            if (sendL9_flag == false)
            {
                AllLightDisable_CheckBox();

                sendDataToMobileRobot("L9", 'n');
                sendL9_flag = true;
                timer_SprawdzKomendeUART.Stop(); timer_SprawdzKomendeUART.Start();
            }
        }

        private void Timer_Migacze_Tick(object sender, EventArgs e)
        {
            if (StanTimerMigacze == true)
            {
                if (StanSwiatlaAwaryjne == true)
                {
                    pictureBox_SwiatlaAwaryjne.Image = Properties.Resources._007_hazard_red;
                }
                if (StanMigaczLewy == true || StanSwiatlaAwaryjne == true)
                {
                    pictureBox_MigaczLewy.Image = Properties.Resources.arrow_50_left_green;
                }
                if (StanMigaczPrawy == true || StanSwiatlaAwaryjne == true)
                {
                    pictureBox_MigaczPrawy.Image = Properties.Resources.arrow_50_right_green;
                }
                StanTimerMigacze = false;
            }
            else
            {
                if (StanSwiatlaAwaryjne == true)
                {
                    pictureBox_SwiatlaAwaryjne.Image = Properties.Resources._007_hazard;
                }
                if (StanMigaczLewy == true || StanSwiatlaAwaryjne == true)
                {
                    pictureBox_MigaczLewy.Image = Properties.Resources.arrow_50_left_gray;
                }
                if (StanMigaczPrawy == true || StanSwiatlaAwaryjne == true)
                {
                    pictureBox_MigaczPrawy.Image = Properties.Resources.arrow_50_right_gray;
                }
                StanTimerMigacze = true;
            }
        }
        
        private void StartStopTimerMigacze()
        {
            if (EnableTimerMigacze > 0)
            {
                Timer_Migacze.Enabled = true;
            }
            else if (EnableTimerMigacze == 0)
            {
                Timer_Migacze.Enabled = false;
            }
        }

        void GetAvailablePorts()    //Funkcja do odczytu dostepnych portow COM
        {
            String[] ports = SerialPort.GetPortNames();     //Przypisanie do tablicy znakow dostepnych nazw portow szeregowych
            comboBox_PortName.Items.AddRange(ports);        //Dodanie dostepnych portow do listy rozwijalnej
        }

        private void SerialPort1_DataReceived(object sender, SerialDataReceivedEventArgs e)
        {
            try
            {
                string readData = sp.ReadExisting();    //Odczyt calego bufora odebranych danych
                if (readData != null)                   //Jak bufor nie jest pusty to uaktualnij ciag odebranych znakow
                {
                    readDataBuffor += readData;
                }
            }
            catch
            {

            }
        }

        private void Button_UARTWyczyscReceived_Click(object sender, EventArgs e)
        {
            listBox_UARTReceivedSendData.Items.Clear();
        }

        private void Button_UARTSend_Click(object sender, EventArgs e)
        {
            if (string.IsNullOrEmpty(textBox_UART_SendData.Text) || string.IsNullOrWhiteSpace(textBox_UART_SendData.Text))  //Sprawdzam czy pole tekstowe jest puste lub zawiera biale znaki
            {
                //listBox_UARTReceivedSendData.ForeColor = Color.Red;
                listBox_UARTReceivedSendData.Items.Add("Nie wpisano znaków do wysłania lub znakiem jest SPACJA!");  //Ostrzerzenie o gowno burzy
            }
            else
            {
                try
                {
                    sp.Write(textBox_UART_SendData.Text + "\n");
                    listBox_UARTReceivedSendData.Items.Add("Wysłano: '" + textBox_UART_SendData.Text + "'");
                }
                catch (Exception ex)
                {
                    listBox_UARTReceivedSendData.Items.Add("Wysyłanie nie powiodło się! " + ex.Message);
                }
            }
        }

        private void Button_UARTWyczyscSend_Click(object sender, EventArgs e)
        {
            textBox_UART_SendData.Clear();
        }
        
        public void Button_Con_DisconRS232_Click(object sender, EventArgs e)
        {
            if (sp.IsOpen)
            {
                Button_DisconnectRS232_Click(sender, e);
            }
            else
            {
                Button_ConnectRS232_Click(sender, e);
            }
        }

        public void Button_ConnectRS232_Click(object sender, EventArgs e)
        {
            //StatusPolaczeniaRS232.Text = "Łączę";
            //StatusPolaczeniaRS232.ForeColor = Color.Orange;

            Button_Con_DisconRS232.Enabled = false;
            Button_ConnectRS232.Enabled = false;

            //listBox_UARTReceivedSendData.Items.Add("Łączenie.");

            try
            {
                //Konfiguracja portu szeregowego
                sp.PortName = comboBox_PortName.Text;                                           //Nazwa Portu
                sp.BaudRate = Convert.ToInt32(comboBox_BaudRate.Text);                          //Predkosc
                sp.DataBits = Convert.ToInt32(comboBox_DataBits.Text);                          //Ilosc danych
                sp.Parity = (Parity)Enum.Parse(typeof(Parity), comboBox_Parity.Text);           //Bit parzystosci
                sp.StopBits = (StopBits)Enum.Parse(typeof(StopBits), comboBox_StopBits.Text);   //Bit stopu
                sp.ReadTimeout = 3000;                                                          //Timeout

                sp.DataReceived += SerialPort1_DataReceived;

                sp.Open();  //Otwarcie portu

                /*      Zakłądka "Sterowanie reczne"     */
                Button_GearManual.Enabled = true;
                Button_GearAutomatic.Enabled = true;
                AllGearDisable_Button();

                CheckBox_SwiatlaDzienne.Enabled = true;
                CheckBox_SwiatlaPostojowe.Enabled = true;
                CheckBox_SwiatlaKrotkie.Enabled = true;
                CheckBox_SwiatlaDlugie.Enabled = true;
                CheckBox_MigaczLewy.Enabled = true;
                CheckBox_MigaczPrawy.Enabled = true;
                CheckBox_SwiatlaAwaryjne.Enabled = true;
                CheckBox_Klakson.Enabled = true;
                CheckBox_SwiatlaPrzeciwmgloweTylne.Enabled = true;
                CheckBox_SwiatlaPrzeciwmglowePrzednie.Enabled = true;

                /*      Zakłądka "Sterowanie autonomiczne"     */
                Button_Algorytm1.Enabled = true;
                Button_Algorytm2.Enabled = true;
                Button_Algorytm3.Enabled = true;
                Button_TestKamera.Enabled = true;

                /*      Zakłądka "Ustawienia"     */
                Button_ConnectRS232.Enabled = false;
                Button_DisconnectRS232.Enabled = true;
                textBox_UART_SendData.Enabled = true;
                Button_UARTSend.Enabled = true;

                comboBox_PortName.Enabled = false;
                comboBox_BaudRate.Enabled = false;
                comboBox_DataBits.Enabled = false;
                comboBox_Parity.Enabled = false;
                comboBox_StopBits.Enabled = false;

                Button_Con_DisconRS232.Text = "Rozłącz";

                StatusPolaczeniaRS232.Text = "Połączony";
                StatusPolaczeniaRS232.ForeColor = Color.LimeGreen;
                
                pictureBox_miniaturkaZdjecia.Image = Properties.Resources.Foto_robota_miniaturka;

                listBox_UARTReceivedSendData.Items.Add("Port został otwarty.");
            }
            catch (Exception ex)
            {
                Button_Con_DisconRS232.Text = "Połącz";
                StatusPolaczeniaRS232.Text = "Rozłączony";
                StatusPolaczeniaRS232.ForeColor = Color.Red;
                pictureBox_miniaturkaZdjecia.Image = Properties.Resources.Foto_robota_miniaturka_gray;

                listBox_UARTReceivedSendData.Items.Add("Port nie został otwarty.");

                MessageBox.Show("Błąd otwarcia portu szeregowego! Sprawdź ustawienia lub zasilanie robota mobilnego. " + ex.Message, "Błąd otwarcia portu", MessageBoxButtons.OK, MessageBoxIcon.Error);
                Button_ConnectRS232.Enabled = true;
            }
            finally
            {
                Button_Con_DisconRS232.Enabled = true;
            }
        }

        public void Button_DisconnectRS232_Click(object sender, EventArgs e)
        {
            sp.Close();

            readDataBuffor = null; newReadDataBuffor = null; oldReadDataBuffor = null;   //Wyzerowanie buforow odbiorczych

            /*      Zakłądka "Sterowanie reczne"     */
            SpeedMobileRobot = 0;
            GaugeSpeedometer.Value = SpeedMobileRobot;
            textBox3_Speed.Text = "0";
            TempCPUMobileRobot = 0;
            GaugeTempCPU.Value = TempCPUMobileRobot;
            textBox_TempCPU.Text = "--,- 'C";
            
            Button_GearManual.BackColor = Color.Transparent;
            Button_GearManual.Enabled = false;
            Button_GearAutomatic.BackColor = Color.Transparent;
            Button_GearAutomatic.Enabled = false;
            AllGearDisable_Button();
            ClearGearBackColor();
            Button_ControlFromKeyboard.BackColor = Color.Gainsboro;
            Button_ControlFromKeyboard.Enabled = false;
            ClearDirectionInfoBackColor();

            CheckBox_SwiatlaDzienne.Enabled = false;
            CheckBox_SwiatlaPostojowe.Enabled = false;
            CheckBox_SwiatlaKrotkie.Enabled = false;
            CheckBox_SwiatlaDlugie.Enabled = false;
            CheckBox_MigaczLewy.Enabled = false;
            CheckBox_MigaczPrawy.Enabled = false;
            CheckBox_SwiatlaAwaryjne.Enabled = false;
            CheckBox_Klakson.Enabled = false;
            CheckBox_SwiatlaPrzeciwmgloweTylne.Enabled = false;
            CheckBox_SwiatlaPrzeciwmglowePrzednie.Enabled = false;

            /*      Zakłądka "Sterowanie autonomiczne"     */
            Button_WyslijDaneDoRobota.Enabled = false;
            Button_StartAlgorytm.Enabled = false;
            Button_StopAlgorytm.Enabled = false;
            Button_Algorytm1.Enabled = false;
            Button_Algorytm2.Enabled = false;
            Button_Algorytm3.Enabled = false;
            Button_Algorytm1.BackColor = Color.Transparent;
            Button_Algorytm2.BackColor = Color.Transparent;
            Button_Algorytm3.BackColor = Color.Transparent;
            wybranyAlgorytm = '0';
            ClearTextBoxsPktStart();
            ClearTextBoxsPktStop();
            wyslanoDanePktDoRobota = false;
            sendPWA_flag = false;
            sendPSX_flag = false;
            sendPSY_flag = false;
            sendPSO_flag = false;
            sendPGX_flag = false;
            sendPGY_flag = false;
            sendPGO_flag = false;
            groupBox_DaneStartowe.Visible = false;
            groupBox_DaneKoncowe.Visible = false;
            groupBox_ParamAlgMotorSchemas.Visible = false;
            groupBox_ParamAlgSubsumption.Visible = false;

            Button_TestKamera.Text = "OFF";
            Button_TestKamera.Enabled = false;
            textBox_PoleObiektu.Text = "-";
            Button_WyslijNowePoleObiektu.Enabled = false;
            textBox_NowePoleObiektu.Text = "0";
            textBox_NowePoleObiektu.BackColor = Color.White;
            textBox_NowePoleObiektu.Enabled = false;

            WyczyscWykresPredkosci();
            WyczyscSiatkeRadaru();
            RysowanieOsiWspolrzednych(xStart, yStart, xStop, yStop);
            WyczyscOknoZachowanieAlgorytmu();
            groupBox_ZachowanieAlgorytmu.Text = "";
            /*      Zakłądka "Ustawienia"     */
            Button_ConnectRS232.Enabled = true;
            Button_DisconnectRS232.Enabled = false;
            Button_UARTSend.Enabled = false;
            comboBox_PortName.Enabled = true;
            comboBox_BaudRate.Enabled = true;
            comboBox_DataBits.Enabled = true;
            comboBox_Parity.Enabled = true;
            comboBox_StopBits.Enabled = true;
            textBox_UART_SendData.Enabled = false;

            Button_Con_DisconRS232.Text = "Połącz";

            StatusPolaczeniaRS232.Text = "Rozłączony";
            StatusPolaczeniaRS232.ForeColor = Color.Red;

            pictureBox_miniaturkaZdjecia.Image = Properties.Resources.Foto_robota_miniaturka_gray;

            listBox_UARTReceivedSendData.Items.Add("Port został zamknety.");
        }

        void ClearTextBoxsPktStart()
        {
            xStart = 0.0; yStart = 0.0; fiStart = 0;
            textBox_xPktStart.Enabled = false;
            textBox_xPktStart.BackColor = Color.White;
            textBox_xPktStart.Text = Convert.ToString(xStart);
            textBox_yPktStart.Enabled = false;
            textBox_yPktStart.BackColor = Color.White;
            textBox_yPktStart.Text = Convert.ToString(yStart);
            textBox_fiPktStart.Enabled = false;
            textBox_fiPktStart.BackColor = Color.White;
            textBox_fiPktStart.Text = Convert.ToString(fiStart);
        }
        void ClearTextBoxsPktStop()
        {
            xStop = 0.0; yStop = 0.0; fiStop = 0;
            textBox_xPktStop.Enabled = false;
            textBox_xPktStop.BackColor = Color.White;
            textBox_xPktStop.Text = Convert.ToString(xStop);
            textBox_yPktStop.Enabled = false;
            textBox_yPktStop.BackColor = Color.White;
            textBox_yPktStop.Text = Convert.ToString(yStop);
            textBox_fiPktStop.Enabled = false;
            textBox_fiPktStop.BackColor = Color.White;
            textBox_fiPktStop.Text = Convert.ToString(fiStop);
        }
        void AllGearEnable_Button()
        {
            Button_GearN.Invoke(new Action(delegate () { Button_GearN.Enabled = true; }));
            Button_Gear1.Invoke(new Action(delegate () { Button_Gear1.Enabled = true; }));
            Button_Gear2.Invoke(new Action(delegate () { Button_Gear2.Enabled = true; }));
            Button_Gear3.Invoke(new Action(delegate () { Button_Gear3.Enabled = true; }));
            Button_Gear4.Invoke(new Action(delegate () { Button_Gear4.Enabled = true; }));
            Button_Gear5.Invoke(new Action(delegate () { Button_Gear5.Enabled = true; }));
            Button_Gear6.Invoke(new Action(delegate () { Button_Gear6.Enabled = true; }));
            Button_GearR.Invoke(new Action(delegate () { Button_GearR.Enabled = true; }));
        }
        void AllGearDisable_Button()
        {
            Button_GearN.Invoke(new Action(delegate () { Button_GearN.Enabled = false; }));
            Button_Gear1.Invoke(new Action(delegate () { Button_Gear1.Enabled = false; }));
            Button_Gear2.Invoke(new Action(delegate () { Button_Gear2.Enabled = false; }));
            Button_Gear3.Invoke(new Action(delegate () { Button_Gear3.Enabled = false; }));
            Button_Gear4.Invoke(new Action(delegate () { Button_Gear4.Enabled = false; }));
            Button_Gear5.Invoke(new Action(delegate () { Button_Gear5.Enabled = false; }));
            Button_Gear6.Invoke(new Action(delegate () { Button_Gear6.Enabled = false; }));
            Button_GearR.Invoke(new Action(delegate () { Button_GearR.Enabled = false; }));
        }
        void ClearGearBackColor()
        {
            Button_GearN.BackColor = Color.Transparent;
            Button_Gear1.BackColor = Color.Transparent;
            Button_Gear2.BackColor = Color.Transparent;
            Button_Gear3.BackColor = Color.Transparent;
            Button_Gear4.BackColor = Color.Transparent;
            Button_Gear5.BackColor = Color.Transparent;
            Button_Gear6.BackColor = Color.Transparent;
            Button_GearR.BackColor = Color.Transparent;
        }
        void ClearDirectionInfoBackColor()
        {
            pictureBox_MotionForward.BackColor = Color.Gainsboro;
            pictureBox_MotionReverse.BackColor = Color.Gainsboro;
            pictureBox_MotionLeft.BackColor    = Color.Gainsboro;
            pictureBox_MotionRight.BackColor   = Color.Gainsboro;
            pictureBox_MotionStop .BackColor   = Color.Gainsboro;
        }
        void SelectActiveGearBackColor(char gear)
        {
            if (gear == 'N')
            {
                Button_GearN.BackColor = Color.LimeGreen;
                Button_GearN.Invoke(new Action(delegate () { Button_GearN.Enabled = false; }));
            }
            else
            {
                Button_GearN.BackColor = Color.Transparent;
                Button_GearN.Invoke(new Action(delegate () { Button_GearN.Enabled = true; }));
            }

            if (gear == '1')
            {
                Button_Gear1.BackColor = Color.LimeGreen;
                Button_Gear1.Invoke(new Action(delegate () { Button_Gear1.Enabled = false; }));
            }
            else
            {
                Button_Gear1.BackColor = Color.Transparent;
                Button_Gear1.Invoke(new Action(delegate () { Button_Gear1.Enabled = true; }));
            }

            if (gear == '2')
            {
                Button_Gear2.BackColor = Color.LimeGreen;
                Button_Gear2.Invoke(new Action(delegate () { Button_Gear2.Enabled = false; }));
            }
            else
            {
                Button_Gear2.BackColor = Color.Transparent;
                Button_Gear2.Invoke(new Action(delegate () { Button_Gear2.Enabled = true; }));
            }

            if (gear == '3')
            {
                Button_Gear3.BackColor = Color.LimeGreen;
                Button_Gear3.Invoke(new Action(delegate () { Button_Gear3.Enabled = false; }));
            }
            else
            {
                Button_Gear3.BackColor = Color.Transparent;
                Button_Gear3.Invoke(new Action(delegate () { Button_Gear3.Enabled = true; }));
            }
            if (gear == '4')
            {
                Button_Gear4.BackColor = Color.LimeGreen;
                Button_Gear4.Invoke(new Action(delegate () { Button_Gear4.Enabled = false; }));
            }
            else
            {
                Button_Gear4.BackColor = Color.Transparent;
                Button_Gear4.Invoke(new Action(delegate () { Button_Gear4.Enabled = true; }));
            }
            if (gear == '5')
            {
                Button_Gear5.BackColor = Color.LimeGreen;
                Button_Gear5.Invoke(new Action(delegate () { Button_Gear5.Enabled = false; }));
            }
            else
            {
                Button_Gear5.BackColor = Color.Transparent;
                Button_Gear5.Invoke(new Action(delegate () { Button_Gear5.Enabled = true; }));
            }
            if (gear == '6')
            {
                Button_Gear6.BackColor = Color.LimeGreen;
                Button_Gear6.Invoke(new Action(delegate () { Button_Gear6.Enabled = false; }));
            }
            else
            {
                Button_Gear6.BackColor = Color.Transparent;
                Button_Gear6.Invoke(new Action(delegate () { Button_Gear6.Enabled = true; }));
            }
            if (gear == 'R')
            {
                Button_GearR.BackColor = Color.LimeGreen;
                Button_GearR.Invoke(new Action(delegate () { Button_GearR.Enabled = false; }));
            }
            else
            {
                Button_GearR.BackColor = Color.Transparent;
                Button_GearR.Invoke(new Action(delegate () { Button_GearR.Enabled = true; }));
            }
        }

        private void PomocSterowanieReczne_Click(object sender, EventArgs e)    //Wywoluje okno pomocy
        {
            new PomocSterowanieReczne().ShowDialog();
        }
    }
}
