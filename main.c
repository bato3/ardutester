/* ************************************************************************
 *
 *   main part
 *
 *   (c) 2012-2013 by Markus Reschke
 *   based on code from Markus Frejek and Karl-Heinz K�bbeler
 *
 * ************************************************************************ */

/*
 *  local constants
 */

/* source management */
#define MAIN_C

/*
 *  include header files
 */

/* local includes */
#include "config.h"    /* global configuration */
#include "common.h"    /* common header file */
#include "variables.h" /* global variables */
#include "LCD.h"       /* LCD module */
#include "functions.h" /* external functions */

/*
 *  local variables
 */

/* programm control */
uint8_t RunsPassed; /* counter for successful measurements */
uint8_t RunsMissed; /* counter for failed/missed measurements */

/* ************************************************************************
 *   output found components
 * ************************************************************************ */

/*
 *  show failed test
 */

void ShowFail(void)
{
    /* display info */
    lcd_fixed_string(Failed1_str); /* display: No component */
    lcd_line(2);                   /* move to line #2 */
    lcd_fixed_string(Failed2_str); /* display: found!*/

    /* display numbers of diodes found */
    if (DiodesFound > 0) /* diodes found */
    {
        lcd_space();                    /* display space */
        lcd_data(DiodesFound + '0');    /* display number of diodes found */
        lcd_fixed_string(Diode_AC_str); /* display: -|>|- */
    }

    RunsMissed++;   /* increase counter */
    RunsPassed = 0; /* reset counter */
}

/*
 *  show error
 */

void ShowError()
{
    if (CompType == TYPE_DISCHARGE) /* discharge failed */
    {
        lcd_fixed_string(DischargeFailed_str); /* display: Battery? */

        /* display probe number and remaining voltage */
        lcd_line(2);
        lcd_testpin(Error.Probe);
        lcd_data(':');
        lcd_space();
        DisplayValue(Error.U, -3, 'V');
    }
}

/*
 *  display Uf of a diode
 */

void ShowDiode_Uf(Diode_Type *Diode)
{
    /* sanity check */
    if (Diode == NULL)
        return;

    /* display Vf */
    DisplayValue(Diode->V_f, -3, 'V');
}

/*
 *  display capacitance of a diode
 */

void ShowDiode_C(Diode_Type *Diode)
{
    /* sanity check */
    if (Diode == NULL)
        return;

    /* get capacitance (opposite of flow direction) */
    MeasureCap(Diode->C, Diode->A, 0);

    /* and show capacitance */
    DisplayValue(Caps[0].Value, Caps[0].Scale, 'F');
}

/*
 *  show diode
 */

void ShowDiode(void)
{
    Diode_Type *D1;        /* pointer to diode #1 */
    Diode_Type *D2 = NULL; /* pointer to diode #2 */
    uint8_t CFlag = 1;     /* capacitance display flag */
    uint8_t A = 5;         /* ID of common anode */
    uint8_t C = 5;         /* ID of common cothode */

    D1 = &Diodes[0]; /* pointer to first diode */

    /*
     *  figure out which diodes to display
     */

    if (DiodesFound == 1) /* single diode */
    {
        C = D1->C; /* make anode first pin */
    }
    else if (DiodesFound == 2) /* two diodes */
    {
        D2 = D1;
        D2++; /* pointer to second diode */

        if (D1->A == D2->A) /* common anode */
        {
            A = D1->A; /* save common anode */
        }
        else if (D1->C == D2->C) /* common cathode */
        {
            C = D1->C; /* save common cathode */
        }
        else if ((D1->A == D2->C) && (D1->C == D2->A)) /* anti-parallel */
        {
            A = D1->A; /* anode and cathode */
            C = A;     /* are the same */
            CFlag = 0; /* disable display of capacitance */
        }
    }
    else if (DiodesFound == 3) /* three diodes */
    {
        uint8_t n;
        uint8_t m;

        /*
         *  Two diodes in series are additionally detected as third big diode:
         *  - Check for any possible way of 2 diodes be connected in series.
         *  - Only once the cathode of diode #1 matches the anode of diode #2.
         */

        for (n = 0; n <= 2; n++) /* loop for first diode */
        {
            D1 = &Diodes[n]; /* get pointer of first diode */

            for (m = 0; m <= 2; m++) /* loop for second diode */
            {
                D2 = &Diodes[m]; /* get pointer of second diode */

                if (n != m) /* don't check same diode :-) */
                {
                    if (D1->C == D2->A) /* got match */
                    {
                        n = 5; /* end loops */
                        m = 5;
                    }
                }
            }
        }

        if (n < 5)
            D2 = NULL; /* no match found */
        C = D1->C;     /* cathode of first diode */
        A = 3;         /* in series mode */
    }
    else /* to much diodes */
    {
        D1 = NULL;  /* don't display any diode */
        ShowFail(); /* and tell user */
        return;
    }

    /*
     *  display pins
     */

    /* first Diode */
    if (A < 3)
        lcd_testpin(D1->C); /* common anode */
    else
        lcd_testpin(D1->A); /* common cathode */

    if (A < 3)
        lcd_fixed_string(Diode_CA_str); /* common anode */
    else
        lcd_fixed_string(Diode_AC_str); /* common cathode */

    if (A < 3)
        lcd_testpin(A); /* common anode */
    else
        lcd_testpin(C); /* common cathode */

    if (D2) /* second diode */
    {
        if (A <= 3)
            lcd_fixed_string(Diode_AC_str); /* common anode or in series */
        else
            lcd_fixed_string(Diode_CA_str); /* common cathode */

        if (A == C)
            lcd_testpin(D2->A); /* anti parallel */
        else if (A <= 3)
            lcd_testpin(D2->C); /* common anode or in series */
        else
            lcd_testpin(D2->A); /* common cathode */
    }

    /*
     *  display Uf (forward voltage) and capacitance
     */

    /* Uf */
    lcd_line(2);              /* go to line #2 */
    lcd_fixed_string(Vf_str); /* display: Vf= */
    ShowDiode_Uf(D1);         /* first diode */
    lcd_space();
    if (D2 == NULL)
    {
        /* display low current Uf if it's quite low (Ge/Schottky diode) */
        if (D1->V_f2 < 250)
        {
            lcd_data('(');
            DisplayValue(D1->V_f2, 0, 0);
            lcd_data(')');
        }
    }
    else
    {
        ShowDiode_Uf(D2); /* second diode (optional) */
    }

    /* capacitance */
    if (CFlag == 1)
    {
        TestKey(3000, 11); /* next page */
        lcd_clear_line(2); /* only change line #2 */

        lcd_fixed_string(DiodeCap_str); /* display: C= */
        ShowDiode_C(D1);                /* first diode */
        lcd_space();
        ShowDiode_C(D2); /* second diode (optional) */
    }
}

/*
 *  show BJT
 */

void ShowBJT(void)
{
    Diode_Type *Diode;     /* pointer to diode */
    unsigned char *String; /* display string pointer */
    uint8_t Counter;       /* counter */
    unsigned int Vf;       /* forward voltage U_be */
    signed int Slope;      /* slope of forward voltage */

    /* display type */
    if (CompType == TYPE_NPN) /* NPN */
        String = (unsigned char *)NPN_str;
    else /* PNP */
        String = (unsigned char *)PNP_str;

    lcd_fixed_string(String); /* display: NPN / PNP */

    /* protections diodes */
    if (DiodesFound > 2) /* transistor is a set of two diodes :-) */
    {
        lcd_space();
        if (CompType == TYPE_NPN) /* NPN */
            String = (unsigned char *)Diode_AC_str;
        else /* PNP */
            String = (unsigned char *)Diode_CA_str;

        lcd_fixed_string(String); /* display: -|>|- / -|<|- */
    }

    /* display pins */
    lcd_space();
    lcd_fixed_string(EBC_str); /* display: EBC= */
    lcd_testpin(BJT.E);        /* display emitter pin */
    lcd_testpin(BJT.B);        /* display base pin */
    lcd_testpin(BJT.C);        /* display collector pin */

    /* display hfe */
    lcd_line(2);               /* move to line #2 */
    lcd_fixed_string(hfe_str); /* display: B= */
    DisplayValue(BJT.hfe, 0, 0);

    /* display Uf (forward voltage) */
    Diode = &Diodes[0]; /* get pointer of first diode */
    Counter = 0;
    while (Counter < DiodesFound) /* check all diodes */
    {
        /* if the diode matches the transistor */
        if (((Diode->A == BJT.B) &&
             (Diode->C == BJT.E) &&
             (CompType == TYPE_NPN)) ||
            ((Diode->A == BJT.E) &&
             (Diode->C == BJT.B) &&
             (CompType == TYPE_PNP)))
        {
            /* not enough space on LCD for large hfe and Vf */
            if (BJT.hfe < 1000) /* small hfe */
            {
                lcd_space(); /* display space */
            }
            else /* line to short */
            {
                TestKey(3000, 11); /* next page */
                lcd_clear_line(2);
            }

            lcd_fixed_string(Vf_str); /* display: Vf= */

            /*
             *  Vf is quite linear for a logarithmicly scaled I_b.
             *  So we may interpolate the Vf values of low and high test current
             *  measurements for a virtual test current. Low test current is 10�A
             *  and high test current is 7mA. That's a logarithmic scale of
             *  3 decades.
             */

            /* calculate slope for one decade */
            Slope = Diode->V_f - Diode->V_f2;
            Slope /= 3;

            /* select Vf based on hfe */
            if (BJT.hfe < 100) /* low hfe */
            {
                /*
                 *  BJTs with low hfe are power transistors and need a large I_b
                 *  to drive the load. So we simply take Vf of the high test current
                 *  measurement (7mA).
                 */

                Vf = Diode->V_f;
            }
            else if (BJT.hfe < 250) /* mid-range hfe */
            {
                /*
                 *  BJTs with a mid-range hfe are signal transistors and need
                 *  a small I_b to drive the load. So we interpolate Vf for
                 *  a virtual test current of about 1mA.
                 */

                Vf = Diode->V_f - Slope;
            }
            else /* high hfe */
            {
                /*
                 *  BJTs with a high hfe are small signal transistors and need
                 *  only a very small I_b to drive the load. So we interpolate Vf
                 *  for a virtual test current of about 0.1mA.
                 */

                Vf = Diode->V_f2 + Slope;
            }

            DisplayValue(Vf, -3, 'V');
            Counter = DiodesFound; /* end loop */
        }
        else
        {
            Counter++; /* increase counter */
            Diode++;   /* next one */
        }
    }
}

/*
 *  show FET
 */

void ShowFET(void)
{
    uint8_t Data; /* temp. data */

    /* display type */
    if (CompType & TYPE_MOSFET)    /* MOSFET */
        lcd_fixed_string(MOS_str); /* display: MOS */
    else                           /* JFET */
        lcd_data('J');             /* display: J */
    lcd_fixed_string(FET_str);     /* display: FET */

    /* display channel type */
    lcd_space();
    if (CompType & TYPE_N_CHANNEL) /* n-channel */
        Data = 'N';
    else /* p-channel */
        Data = 'P';

    lcd_data(Data);                /* display: N / P */
    lcd_fixed_string(Channel_str); /* display: -ch */

    /* display mode */
    if (CompType & TYPE_MOSFET) /* MOSFET */
    {
        lcd_space();
        if (CompType & TYPE_ENHANCEMENT) /* enhancement mode */
            lcd_fixed_string(Enhancement_str);
        else /* depletion mode */
            lcd_fixed_string(Depletion_str);
    }

    /* pins */
    lcd_line(2);               /* move to line #2 */
    lcd_fixed_string(GDS_str); /* display: GDS= */
    lcd_testpin(FET.G);        /* display gate pin */
    lcd_testpin(FET.D);        /* display drain pin */
    lcd_testpin(FET.S);        /* display source pin */

    /* extra data for MOSFET in enhancement mode */
    if (CompType & (TYPE_ENHANCEMENT | TYPE_MOSFET))
    {
        /* protection diode */
        if (DiodesFound > 0)
        {
            lcd_space();               /* display space */
            lcd_data(LCD_CHAR_DIODE1); /* display diode symbol */
        }

        TestKey(3000, 11); /* next page */
        lcd_clear();

        /* gate threshold voltage */
        lcd_fixed_string(Vth_str);       /* display: Vth */
        DisplayValue(FET.V_th, -3, 'V'); /* display V_th in mV */

        lcd_line(2);

        /* display gate capacitance */
        lcd_fixed_string(GateCap_str); /* display: Cgs= */
        MeasureCap(FET.G, FET.S, 0);   /* measure capacitance */
        /* display value and unit */
        DisplayValue(Caps[0].Value, Caps[0].Scale, 'F');
    }
}

/*
 *   show special components like Thyristor and Triac
 */

void ShowSpecial(void)
{
    /* display component type */
    if (CompFound == COMP_THYRISTOR)
    {
        lcd_fixed_string(Thyristor_str); /* display: thyristor */
    }
    else if (CompFound == COMP_TRIAC)
    {
        lcd_fixed_string(Triac_str); /* display: triac */
    }

    /* display pins */
    lcd_line(2);               /* move to line #2 */
    lcd_fixed_string(GAK_str); /* display: GAK */
    lcd_testpin(BJT.B);        /* display gate pin */
    lcd_testpin(BJT.C);        /* display anode pin */
    lcd_testpin(BJT.E);        /* display cathode pin */
}

/*
 *  show resistor
 */

void ShowResistor(void)
{
    Resistor_Type *R1; /* pointer to resistor #1 */
    Resistor_Type *R2; /* pointer to resistor #2 */
    uint8_t Pin;       /* ID of common pin */

    R1 = &Resistors[0]; /* pointer to first resistor */

    if (ResistorsFound == 1) /* single resistor */
    {
        R2 = NULL;   /* disable second resistor */
        Pin = R1->A; /* make B the first pin */
    }
    else /* multiple resistors */
    {
        R2 = R1;
        R2++; /* pointer to second resistor */

        if (ResistorsFound == 3) /* three resistors */
        {
            Resistor_Type *Rmax; /* pointer to largest resistor */

            /*
             *  3 resistors mean 2 single resistors and both resitors in series.
             *  So we have to single out that series resistor by finding the
             *  largest resistor.
             */

            Rmax = R1; /* starting point */
            for (Pin = 1; Pin <= 2; Pin++)
            {
                if (CmpValue(R2->Value, R2->Scale, Rmax->Value, Rmax->Scale) == 1)
                {
                    Rmax = R2; /* update largest one */
                }

                R2++; /* next one */
            }

            /* get the two smaller resistors */
            if (R1 == Rmax)
                R1++;
            R2 = R1;
            R2++;
            if (R2 == Rmax)
                R2++;
        }

        /* find common pin of both resistors */
        if ((R1->A == R2->A) || (R1->A == R2->B))
            Pin = R1->A;
        else
            Pin = R1->B;
    }

    /*
     *  display the pins
     */

    /* first resistor */
    if (R1->A != Pin)
        lcd_testpin(R1->A);
    else
        lcd_testpin(R1->B);
    lcd_fixed_string(Resistor_str);
    lcd_testpin(Pin);

    if (R2) /* second resistor */
    {
        lcd_fixed_string(Resistor_str);
        if (R2->A != Pin)
            lcd_testpin(R2->A);
        else
            lcd_testpin(R2->B);
    }

    /*
     *  display the values
     */

    /* first resistor */
    lcd_line(2);
    DisplayValue(R1->Value, R1->Scale, LCD_CHAR_OMEGA);

    if (R2) /* second resistor */
    {
        lcd_space();
        DisplayValue(R2->Value, R2->Scale, LCD_CHAR_OMEGA);
    }
}

/*
 *  show capacitor
 */

void ShowCapacitor(void)
{
    Capacitor_Type *MaxCap; /* pointer to largest cap */
    Capacitor_Type *Cap;    /* pointer to cap */
    uint8_t Counter;        /* loop counter */

    /* find largest cap */
    MaxCap = &Caps[0]; /* pointer to first cap */
    Cap = MaxCap;

    for (Counter = 1; Counter <= 2; Counter++)
    {
        Cap++; /* next cap */

        if (CmpValue(Cap->Value, Cap->Scale, MaxCap->Value, MaxCap->Scale) == 1)
        {
            MaxCap = Cap;
        }
    }

    /* display largest cap */
    lcd_testpin(MaxCap->A);    /* display pin #1 */
    lcd_fixed_string(Cap_str); /* display capacitor symbol */
    lcd_testpin(MaxCap->B);    /* display pin #2 */
    lcd_line(2);               /* move to line #2 */
    /* and show capacitance */
    DisplayValue(MaxCap->Value, MaxCap->Scale, 'F');
}

/* ************************************************************************
 *   the one and only main()
 * ************************************************************************ */

/*
 *  main function
 */

int main(void)
{
    unsigned int U_Bat; /* voltage of power supply */
    uint8_t Test;       /* test value */

    /*
     *  init
     */

    /* switch on power to keep me alive */
    CONTROL_DDR = (1 << POWER_CTRL);  /* set pin as output */
    CONTROL_PORT = (1 << POWER_CTRL); /* set pin to drive power management transistor */

    /* setup �C */
    MCUCR = (1 << PUD);                   /* disable pull-up resistors globally */
    ADCSRA = (1 << ADEN) | ADC_CLOCK_DIV; /* enable ADC and set clock divider */

    /* catch watchdog */
    Test = (MCUSR & (1 << WDRF)); /* save watchdog flag */
    MCUSR &= ~(1 << WDRF);        /* reset watchdog flag */
    wdt_disable();                /* disable watchdog */

    /*
     *  watchdog was triggered (timeout 2s)
     *  - This is after the �C done a reset driven by the watchdog.
     *  - Does only work if the capacitor at the base of the power management
     *    transistor is large enough to survive a �C reset. Otherwise the
     *    tester simply looses power.
     */

    if (Test)
    {
        lcd_clear();                   /* display was initialized before */
        lcd_fixed_string(Timeout_str); /* display: timeout */
        lcd_line(2);
        lcd_fixed_string(Error_str); /* display: error */
        MilliSleep(2000);            /* give user some time to read */
        CONTROL_PORT = 0;            /* power off myself */
        return 0;                    /* exit program */
    }

    /*
     *  init LCD module and load custom characters
     */

    lcd_init(); /* initialize LCD */

    /* symbols for components */
    lcd_fixed_customchar(DiodeIcon1, LCD_CHAR_DIODE1); /* diode symbol '|>|' */
    lcd_fixed_customchar(DiodeIcon2, LCD_CHAR_DIODE2); /* diode symbol '|<|' */
    lcd_fixed_customchar(CapIcon, LCD_CHAR_CAP);       /* capacitor symbol '||' */
    lcd_fixed_customchar(ResIcon1, LCD_CHAR_RESIS1);   /* resistor symbol '[' */
    lcd_fixed_customchar(ResIcon2, LCD_CHAR_RESIS2);   /* resistor symbol ']' */

/* kyrillish LCD character set lacks omega and � */
#ifdef LCD_CYRILLIC
    lcd_fixed_customchar(OmegaIcon, LCD_CHAR_OMEGA); /* Omega */
    lcd_fixed_customchar(MicroIcon, LCD_CHAR_MICRO); /* � / micro */
#endif

    /* return to normal output */
    lcd_line(1); /* move to line #1 */

    /*
     *  operation mode selection
     */

    Config.SleepMode = SLEEP_MODE_PWR_SAVE; /* default: power save */
    Config.TesterMode = MODE_CONTINOUS;     /* set default mode: continous */

    /* catch long key press */
    if (!(CONTROL_PIN & (1 << TEST_BUTTON))) /* if test button is pressed */
    {
        MilliSleep(300);                         /* wait to catch a long key press */
        if (!(CONTROL_PIN & (1 << TEST_BUTTON))) /* if button is still pressed */
            Config.TesterMode = MODE_AUTOHOLD;   /* set auto-hold mode */
    }

    /* output operation mode */
    lcd_fixed_string(Mode_str);              /* display: tester mode */
    lcd_line(2);                             /* move to line #2 */
    if (Config.TesterMode == MODE_CONTINOUS) /* if continous mode */
        lcd_fixed_string(Continous_str);     /* display: continous */
    else                                     /* if auto-hold mode */
        lcd_fixed_string(AutoHold_str);      /* display: auto-hold */
    MilliSleep(2000);                        /* give user some time to read */

    /*
     *  init variables
     */

    /* cycling */
    RunsMissed = 0;
    RunsPassed = 0;

    /* default offsets and values */
    Config.Samples = ADC_SAMPLES; /* number of ADC samples */
    Config.AutoScale = 1;         /* enable ADC auto scaling */
    Config.RefFlag = 1;           /* no ADC reference set yet */
    LoadAdjust();                 /* load adjustment values */

    wdt_enable(WDTO_2S); /* enable watchdog (timeout 2s) */

    /*
     *  main processing cycle
     */

start:

    /* reset variabels */
    CompFound = COMP_NONE;
    CompType = 0;
    CompDone = 0;
    DiodesFound = 0;
    ResistorsFound = 0;
    BJT.hfe = 0;

    /* reset hardware */
    ADC_DDR = 0; /* set all pins of ADC port as input */
    lcd_clear(); /* clear LCD */

    /* internal bandgap reference */
    Config.U_Bandgap = ReadU(0x0e);       /* dummy read for bandgap stabilization */
    Config.Samples = 200;                 /* do a lot of samples for high accuracy */
    Config.U_Bandgap = ReadU(0x0e);       /* get voltage of bandgap reference */
    Config.Samples = ADC_SAMPLES;         /* set samples back to default */
    Config.U_Bandgap += Config.RefOffset; /* add voltage offset */

    /*
     *  battery check
     */

    /* get current voltage */
    U_Bat = ReadU(5); /* read voltage of ADC5 in mV */

    /*
     *  ADC pin is connected to a voltage divider Rh = 10k and Rl = 3k3.
     *  Ul = (Uin / (Rh + Rl)) * Rl  ->  Uin = (Ul * (Rh + Rl)) / Rl
     *  Uin = (Ul * (10k + 3k3)) / 3k3 = 4 * Ul
     */

    U_Bat *= 4;          /* calculate U_bat (mV) */
    U_Bat += BAT_OFFSET; /* add offset for voltage drop */

    /* display battery voltage */
    lcd_fixed_string(Battery_str); /* display: Bat. */
    lcd_space();
    DisplayValue(U_Bat / 10, -2, 'V'); /* display battery voltage */
    lcd_space();

    /* check limits */
    if (U_Bat < BAT_POOR) /* low level reached */
    {
        lcd_fixed_string(Low_str); /* display: low */
        MilliSleep(2000);          /* let user read info */
        goto power_off;            /* power off */
    }
    else if (U_Bat < BAT_POOR + 1000) /* warning level reached */
    {
        lcd_fixed_string(Weak_str); /* display: weak */
    }
    else /* ok */
    {
        lcd_fixed_string(OK_str); /* display: ok */
    }

    /*
     *  probing
     */

    /* display start of probing */
    lcd_line(2);                   /* move to line #2 */
    lcd_fixed_string(Running_str); /* display: probing... */

    /* try to discharge any connected component */
    DischargeProbes();
    if (CompFound == COMP_ERROR) /* discharge failed */
    {
        goto result; /* skip all other checks */
    }

    /* enter main menu if requested by short-circuiting all probes */
    if (AllProbesShorted() == 3)
    {
        MainMenu(); /* enter mainmenu */
        ;
        goto end; /* new cycle after job is is done */
    }

    /* check all 6 combinations of the 3 probes */
    CheckProbes(TP1, TP2, TP3);
    CheckProbes(TP2, TP1, TP3);
    CheckProbes(TP1, TP3, TP2);
    CheckProbes(TP3, TP1, TP2);
    CheckProbes(TP2, TP3, TP1);
    CheckProbes(TP3, TP2, TP1);

    /* if component might be a capacitor */
    if ((CompFound == COMP_NONE) ||
        (CompFound == COMP_RESISTOR))
    {
        /* tell user to be patient with large caps :-) */
        lcd_clear_line(2);
        lcd_fixed_string(Running_str);
        lcd_space();
        lcd_data('C');

        /* check all possible combinations */
        MeasureCap(TP3, TP1, 0);
        MeasureCap(TP3, TP2, 1);
        MeasureCap(TP2, TP1, 2);
    }

    /*
     *  output test results
     */

result:

    lcd_clear(); /* clear LCD */

    /* call output function based on component type */
    switch (CompFound)
    {
    case COMP_ERROR:
        ShowError();
        goto end;
        break;

    case COMP_DIODE:
        ShowDiode();
        break;

    case COMP_BJT:
        ShowBJT();
        break;

    case COMP_FET:
        ShowFET();
        break;

    case COMP_THYRISTOR:
        ShowSpecial();
        break;

    case COMP_TRIAC:
        ShowSpecial();
        break;

    case COMP_RESISTOR:
        ShowResistor();
        break;

    case COMP_CAPACITOR:
        ShowCapacitor();
        break;

    default: /* no component found */
        ShowFail();
        goto end;
    }

    /* component was found */
    RunsMissed = 0; /* reset counter */
    RunsPassed++;   /* increase counter */

    /*
     *  take care about cycling and power-off
     */

end:

    /* get key press or timeout */
    Test = TestKey((unsigned int)CYCLE_DELAY, 12);

    if (Test == 1) /* short key press */
    {
        /* a second key press triggers extra functions */
        MilliSleep(50);
        Test = TestKey(300, 0);

        if (Test > 0) /* short or long key press */
        {
            MainMenu(); /* enter main menu */
            goto end;   /* re-run cycle control */
        }

        goto start; /* -> next round */
    }
    else if (Test == 2) /* long key press */
    {
        goto power_off; /* -> power off */
    }

    /* check if we should go for another round (continious mode only) */
    if ((RunsMissed < CYCLE_MAX) && (RunsPassed < CYCLE_MAX * 2))
    {
        goto start; /* another round */
    }

power_off:

    /* display feedback (otherwise the user will wait :-) */
    lcd_clear();
    lcd_fixed_string(Version_str); /* display firmware version */
    lcd_line(2);
    lcd_fixed_string(Done_str); /* display: done! */
    MilliSleep(1000);           /* let the user read the text */

    wdt_disable();                      /* disable watchdog */
    CONTROL_PORT &= ~(1 << POWER_CTRL); /* power off myself */

    return 0;
}

/* ************************************************************************
 *   clean-up of local constants
 * ************************************************************************ */

/* source management */
#undef MAIN_C

/* ************************************************************************
 *   EOF
 * ************************************************************************ */
