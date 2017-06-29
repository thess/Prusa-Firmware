#define WELCOME_MSG                         CUSTOM_MENDEL_NAME " pronta."
#define MSG_SD_INSERTED                     "SD inserita"
#define MSG_SD_REMOVED                      "SD rimossa"
#define MSG_MAIN                            "Menu principale"
#define MSG_DISABLE_STEPPERS                "Disabilit motori"
#define MSG_AUTO_HOME                       "Trova origine"
#define MSG_SET_HOME_OFFSETS                "Set home offsets"
#define MSG_SET_ORIGIN                      "Set origin"
#define MSG_COOLDOWN                        "Raffredda"
#define MSG_SWITCH_PS_ON                    "Switch power on"
#define MSG_SWITCH_PS_OFF                   "Switch power off"
#define MSG_MOVE_AXIS                       "Muovi asse"
#define MSG_MOVE_X                          "Muovi X"
#define MSG_MOVE_Y                          "Muovi Y"
#define MSG_MOVE_Z                          "Muovi Z"
#define MSG_MOVE_E                          "Muovi Estrusore"
#define MSG_MOVE_01MM                       "Move 0.1mm"
#define MSG_MOVE_1MM                        "Move 1mm"
#define MSG_MOVE_10MM                       "Move 10mm"
#define MSG_SPEED                           "Velocita"
#define MSG_NOZZLE                          "Ugello"
#define MSG_NOZZLE1                         "Nozzle2"
#define MSG_NOZZLE2                         "Nozzle3"
#define MSG_BED                             "Letto"
#define MSG_FAN_SPEED                       "Velocita vent."
#define MSG_FLOW                            "Flusso"
#define MSG_TEMPERATURE                     "Temperatura"
#define MSG_MOTION                          "Motion"
#define MSG_VOLUMETRIC                      "Filament"
#define MSG_VOLUMETRIC_ENABLED		    	"E in mm3"
#define MSG_STORE_EPROM                     "Store memory"
#define MSG_LOAD_EPROM                      "Load memory"
#define MSG_RESTORE_FAILSAFE                "Restore failsafe"
#define MSG_REFRESH                         "\xF8" "Refresh"
#define MSG_WATCH                           "Schermata info"
#define MSG_TUNE                            "Regola"
#define MSG_PAUSE_PRINT                     "Metti in pausa"
#define MSG_RESUME_PRINT                    "Riprendi stampa"
#define MSG_STOP_PRINT                      "Arresta stampa"
#define MSG_CARD_MENU                       "Stampa da SD"
#define MSG_NO_CARD                         "Nessuna SD"
#define MSG_DWELL                           "Sospensione..."
#define MSG_USERWAIT                        "Attendendo utente"
#define MSG_RESUMING                        "Riprendi stampa"
#define MSG_PRINT_ABORTED                   "Stampa abortita"
#define MSG_NO_MOVE                         "Nessun movimento."
#define MSG_KILLED                          "IN TILT."
#define MSG_STOPPED                         "ARRESTATO."
#define MSG_FILAMENTCHANGE                  "Camb. filamento"
#define MSG_INIT_SDCARD                     "Init. SD card"
#define MSG_CNG_SDCARD                      "Change SD card"
#define MSG_ZPROBE_OUT                      "Z probe out. bed"
#define MSG_POSITION_UNKNOWN                "Home X/Y before Z"
#define MSG_ZPROBE_ZOFFSET                  "Z Offset"
#define MSG_BABYSTEP_X                      "Babystep X"
#define MSG_BABYSTEP_Y                      "Babystep Y"
#define MSG_BABYSTEP_Z                      "Compensazione Z"
#define MSG_ADJUSTZ			    			"Autoregolare Z?"
#define MSG_PICK_Z			    			"Pick print"

#define MSG_SETTINGS                        "Impostazioni"
#define MSG_PREHEAT                         "Preriscalda"
#define MSG_HEATING                         "Riscaldamento..."
#define MSG_SUPPORT 						"Support"
#define MSG_YES								"Si"
#define MSG_NO								"No"
#define MSG_NOT_LOADED 						"Fil. non caricato"
#define MSG_NOT_COLOR 						"Colore non puro"
#define MSG_LOADING_COLOR					"Caricando colore"
#define MSG_CHANGE_SUCCESS					"Cambio riuscito!"
#define MSG_PRESS							"e cliccare manopola"
#define MSG_INSERT_FILAMENT					"Inserire filamento"
#define MSG_CHANGING_FILAMENT				"Cambiando filam."

#define MSG_PLEASE_WAIT						"Aspetta"
#define MSG_PREHEAT_NOZZLE                  "Preris. ugello!"
#define MSG_HEATING_COMPLETE                "Riscald. completo"
#define MSG_BED_HEATING                     "Riscald. letto"
#define MSG_BED_DONE                        "Piatto fatto."
#define MSG_ERROR                        	"ERRORE:"
#define MSG_CORRECTLY						"Cambiato corr.?"
#define MSG_LOADING_FILAMENT				"Caricando filam."
#define MSG_UNLOAD_FILAMENT                 "Scarica filamento"
#define MSG_LOAD_FILAMENT                   "Carica filamento"

#define MSG_SILENT_MODE_ON		    "Modo [silenzioso]"
#define MSG_SILENT_MODE_OFF		    "Mode      [forte]"
#define MSG_AUTO_MODE_ON			"Mode       [auto]"
#define MSG_REBOOT			    "Riavvia stampante"
#define MSG_TAKE_EFFECT			    " per attualizzare"

#define MSG_Enqueing                        "enqueing \""
#define MSG_POWERUP                         "PowerUp"
#define MSG_CONFIGURATION_VER               " Last Updated: "
#define MSG_FREE_MEMORY                     " Free Memory: "
#define MSG_PLANNER_BUFFER_BYTES            "  PlannerBufferBytes: "
#define MSG_OK                              "ok"
#define MSG_ERR_CHECKSUM_MISMATCH           "checksum mismatch, Last Line: "
#define MSG_ERR_NO_CHECKSUM                 "No Checksum with line number, Last Line: "
#define MSG_BEGIN_FILE_LIST                 "Begin file list"
#define MSG_END_FILE_LIST                   "End file list"
#define MSG_M104_INVALID_EXTRUDER           "M104 Invalid extruder "
#define MSG_M105_INVALID_EXTRUDER           "M105 Invalid extruder "
#define MSG_M200_INVALID_EXTRUDER           "M200 Invalid extruder "
#define MSG_M218_INVALID_EXTRUDER           "M218 Invalid extruder "
#define MSG_M221_INVALID_EXTRUDER           "M221 Invalid extruder "
#define MSG_ERR_NO_THERMISTORS              "No thermistors - no temperature"
#define MSG_M109_INVALID_EXTRUDER           "M109 Invalid extruder "
#define MSG_ERR_KILLED                      "Printer halted. kill() called!"
#define MSG_ERR_STOPPED                     "Printer stopped due to errors. Fix the error and use M999 to restart. (Temperature is reset. Set it after restarting)"
#define MSG_RESEND                          "Resend: "
#define MSG_M119_REPORT                     "Reporting endstop status"
#define MSG_ENDSTOP_HIT                     "TRIGGERED"
#define MSG_ENDSTOP_OPEN                    "open"
#define MSG_SD_CANT_OPEN_SUBDIR             "Cannot open subdir"
#define MSG_SD_INIT_FAIL                    "SD init fail"
#define MSG_SD_VOL_INIT_FAIL                "volume.init failed"
#define MSG_SD_OPENROOT_FAIL                "openRoot failed"
#define MSG_SD_CARD_OK                      "SD card ok"
#define MSG_SD_WORKDIR_FAIL                 "workDir open failed"
#define MSG_SD_OPEN_FILE_FAIL               "open failed, File: "
#define MSG_SD_FILE_OPENED                  "File opened: "
#define MSG_SD_FILE_SELECTED                "File selected"
#define MSG_SD_WRITE_TO_FILE                "Writing to file: "
#define MSG_SD_PRINTING_BYTE                "SD printing byte "
#define MSG_SD_NOT_PRINTING                 "Not SD printing"
#define MSG_SD_ERR_WRITE_TO_FILE            "error writing to file"
#define MSG_SD_CANT_ENTER_SUBDIR            "Cannot enter subdir: "
#define MSG_STEPPER_TOO_HIGH                "Steprate too high: "
#define MSG_ENDSTOPS_HIT                    "endstops hit: "
#define MSG_ERR_COLD_EXTRUDE_STOP           " cold extrusion prevented"
#define MSG_BABYSTEPPING_X                  "Babystepping X"
#define MSG_BABYSTEPPING_Y                  "Babystepping Y"
#define MSG_BABYSTEPPING_Z                  "Compensazione Z"
#define MSG_SERIAL_ERROR_MENU_STRUCTURE     "Error in menu structure"

#define MSG_LANGUAGE_NAME		    "Italiano"
#define MSG_LANGUAGE_SELECT		    "Seleziona lingua"
#define MSG_PRUSA3D			    "prusa3d.com"
#define MSG_PRUSA3D_FORUM		    "forum.prusa3d.com"
#define MSG_PRUSA3D_HOWTO		    "howto.prusa3d.com"

#define MSG_SELFTEST_ERROR		    "Autotest negativo"
#define MSG_SELFTEST_PLEASECHECK	    "Verificare:"	
#define MSG_SELFTEST_NOTCONNECTED	    "Non connesso"
#define MSG_SELFTEST_HEATERTHERMISTOR	    "Riscald./Termist."
#define MSG_SELFTEST_BEDHEATER		    "Letto/Riscald."
#define MSG_SELFTEST_WIRINGERROR	    "Errore cablaggio"
#define MSG_SELFTEST_ENDSTOPS		    "Finecorsa (2)"
#define MSG_SELFTEST_MOTOR		    "Motore"
#define MSG_SELFTEST_ENDSTOP		    "Finecorsa"
#define MSG_SELFTEST_ENDSTOP_NOTHIT	    "Finec. fuori por."
#define MSG_SELFTEST_OK			    "Autotest OK"

#define MSG_SELFTEST_FAN					"Prova del ventilator"
#define MSG_SELFTEST_COOLING_FAN			"Vent di stampa ant.?"
#define MSG_SELFTEST_EXTRUDER_FAN        "Vent SX sull'ugello?"
#define MSG_SELFTEST_FAN_YES				"Gira"
#define MSG_SELFTEST_FAN_NO					"Non si gira"

#define MSG_STATS_TOTALFILAMENT		    "Filamento tot:"
#define MSG_STATS_TOTALPRINTTIME	    "Tempo stampa tot:"
#define MSG_STATS_FILAMENTUSED		    "Filamento usato:"
#define MSG_STATS_PRINTTIME		    "Tempo di stampa:"
#define MSG_SELFTEST_START		    "Avvia autotest"
#define MSG_SELFTEST_CHECK_ENDSTOPS	    "Verifica finecorsa"
#define MSG_SELFTEST_CHECK_HOTEND	    "Verifica ugello"  
#define MSG_SELFTEST_CHECK_X		    "Verifica asse X"
#define MSG_SELFTEST_CHECK_Y		    "Verifica asse Y"
#define MSG_SELFTEST_CHECK_Z		    "Verifica asse Z"
#define MSG_SELFTEST_CHECK_BED		    "Verifica letto"
#define MSG_SELFTEST_CHECK_ALLCORRECT	    "Nessun errore"
#define MSG_SELFTEST			    "Autotest"
#define MSG_SELFTEST_FAILED		    "Autotest fallito"
#define MSG_STATISTICS			    "Statistiche"
#define MSG_USB_PRINTING		    "Stampa da USB"
#define MSG_HOMEYZ                          "Calibra Z"
#define MSG_HOMEYZ_PROGRESS                 "Calibrando Z"
#define MSG_HOMEYZ_DONE		            "Calibrazione OK"



#define MSG_SHOW_END_STOPS					"Stato finecorsa"
#define MSG_CALIBRATE_BED					"Calibra XYZ"
#define MSG_CALIBRATE_BED_RESET					"Reset XYZ calibr."

#define MSG_MOVE_CARRIAGE_TO_THE_TOP 				"Calibrazione XYZ. Ruotare la manopola per alzare il carrello Z fino all'altezza massima. Click per terminare."
#define MSG_MOVE_CARRIAGE_TO_THE_TOP_Z 				"Calibrazione Z. Ruotare la manopola per alzare il carrello Z fino all'altezza massima. Click per terminare."

#define MSG_CONFIRM_NOZZLE_CLEAN					"Pulire l'ugello per la calibrazione, poi fare click."
#define MSG_CONFIRM_CARRIAGE_AT_THE_TOP				"I carrelli Z sin/des sono altezza max?"

#define MSG_FIND_BED_OFFSET_AND_SKEW_LINE1			"Ricerca del letto punto di calibraz."
#define MSG_FIND_BED_OFFSET_AND_SKEW_LINE2			" su 4"
#define MSG_IMPROVE_BED_OFFSET_AND_SKEW_LINE1		"Perfezion. il letto punto di calibraz."
#define MSG_IMPROVE_BED_OFFSET_AND_SKEW_LINE2		" su 9"
#define MSG_MEASURE_BED_REFERENCE_HEIGHT_LINE1		"Misurare l'altezza di riferimento del punto di calibrazione"
#define MSG_MEASURE_BED_REFERENCE_HEIGHT_LINE2		" su 9"
#define MSG_FIND_BED_OFFSET_AND_SKEW_ITERATION	"Reiterazione "

#define MSG_BED_SKEW_OFFSET_DETECTION_POINT_NOT_FOUND		"Calibrazione XYZ fallita. Il punto di calibrazione sul letto non e' stato trovato."
#define MSG_BED_SKEW_OFFSET_DETECTION_FITTING_FAILED		"Calibrazione XYZ fallita. Si prega di consultare il manuale."
#define MSG_BED_SKEW_OFFSET_DETECTION_PERFECT			"Calibrazione XYZ OK. Gli assi X/Y sono perpendicolari. Complimenti!"
#define MSG_BED_SKEW_OFFSET_DETECTION_SKEW_MILD			"Calibrazion XYZ corretta. Assi X/Y leggermente storti. Ben fatto!"
#define MSG_BED_SKEW_OFFSET_DETECTION_SKEW_EXTREME		"Calibrazion XYZ corretta. La distorsione verra' automaticamente compensata."
#define MSG_BED_SKEW_OFFSET_DETECTION_FAILED_FRONT_LEFT_FAR	"Calibrazione XYZ fallita. Punto anteriore sinistro non raggiungibile."
#define MSG_BED_SKEW_OFFSET_DETECTION_FAILED_FRONT_RIGHT_FAR	"Calibrazione XYZ fallita. Punto anteriore destro non raggiungibile."
#define MSG_BED_SKEW_OFFSET_DETECTION_FAILED_FRONT_BOTH_FAR	"Calibrazione XYZ fallita. Punti anteriori non raggiungibili."
#define MSG_BED_SKEW_OFFSET_DETECTION_WARNING_FRONT_LEFT_FAR	"Calibrazione XYZ compromessa. Punto anteriore sinistro non raggiungibile."
#define MSG_BED_SKEW_OFFSET_DETECTION_WARNING_FRONT_RIGHT_FAR	"Calibrazione XYZ compromessa. Punto anteriore destro non raggiungibile."
#define MSG_BED_SKEW_OFFSET_DETECTION_WARNING_FRONT_BOTH_FAR	"Calibrazione XYZ compromessa. Punti anteriori non raggiungibili."

#define MSG_BED_LEVELING_FAILED_POINT_LOW				"Livellamento letto fallito.NoRispSensor Residui su ugello? In attesa di reset."
#define MSG_BED_LEVELING_FAILED_POINT_HIGH				"Livellamento letto fallito.Risp sensore troppo prestoIn attesa di reset."
#define MSG_BED_LEVELING_FAILED_PROBE_DISCONNECTED		"Livellamento letto fallito. Sensore discon. o Cavo Dann. In attesa di reset."

#define MSG_NEW_FIRMWARE_AVAILABLE						"Nuova versione del firmware disponibile"
#define MSG_NEW_FIRMWARE_PLEASE_UPGRADE					"Prega aggiorna."

#define MSG_FOLLOW_CALIBRATION_FLOW						"Stampante ancora non calibrata. Si prega di seguire il manuale, capitolo PRIMI PASSI, sezione della calibrazione."
#define MSG_BABYSTEP_Z_NOT_SET							"Distanza tra la punta dell'ugello e la superficie del letto non ancora imposta. Si prega di seguire il manuale, capitolo First steps, sezione First layer calibration."

#define MSG_BED_CORRECTION_MENU							"Correz. liv.letto"
#ifdef MBC_8POINT
#define MSG_BED_CORRECTION_P1		"Punto a [um]"
#define MSG_BED_CORRECTION_P2		"Punto b [um]"
#define MSG_BED_CORRECTION_P3		"Punto c [um]"
#define MSG_BED_CORRECTION_P4		"Punto d [um]"
#define MSG_BED_CORRECTION_P5		"Punto e [um]"
#define MSG_BED_CORRECTION_P6		"Punto f [um]"
#define MSG_BED_CORRECTION_P7		"Punto g [um]"
#define MSG_BED_CORRECTION_P8		"Punto h [um]"
#else
//#define MSG_BED_CORRECTION_LEFT							"Sinistra  [um]"
//#define MSG_BED_CORRECTION_RIGHT						"Destra    [um]"
//#define MSG_BED_CORRECTION_FRONT						"Fronte    [um]"
//#define MSG_BED_CORRECTION_REAR							"Retro     [um]"
#endif
#define MSG_BED_CORRECTION_RESET						"Reset"			

#define MSG_MESH_BED_LEVELING							"Mesh livel. letto"
#define MSG_MENU_CALIBRATION							"Calibrazione"
#define MSG_TOSHIBA_FLASH_AIR_COMPATIBILITY_OFF					"SD card [normal]"
#define MSG_TOSHIBA_FLASH_AIR_COMPATIBILITY_ON					"SD card [FlshAir]"

#define MSG_LOOSE_PULLEY								"Puleggia lenta"
#define MSG_FILAMENT_LOADING_T0							"Inserire filamento nell'estrusore 1. Click per continuare."
#define MSG_FILAMENT_LOADING_T1							"Inserire filamento nell'estrusore 2. Click per continuare."
#define MSG_FILAMENT_LOADING_T2							"Inserire filamento nell'estrusore 3. Click per continuare."
#define MSG_FILAMENT_LOADING_T3							"Inserire filamento nell'estrusore 4. Click per continuare."
#define MSG_CHANGE_EXTR									"Cambio estrusore."

#define MSG_FIL_ADJUSTING								"Filamento in fase di regolazione. Attendere prego."
#define MSG_CONFIRM_NOZZLE_CLEAN_FIL_ADJ				"I filamenti sono regolati. Si prega di pulire l'ugello per la calibrazione. Click per continuare."
#define MSG_CALIBRATE_E									"Calibra E"
#define MSG_E_CAL_KNOB									"Girare la manopola affinche' il segno raggiunga il corpo dell'estrusore. Click per continuare."
#define MSG_MARK_FIL									"Segnare il filamento a 100 mm di distanza dal corpo dell'estrusore. Click per continuare."
#define MSG_CLEAN_NOZZLE_E								"Calibrazione E terminata. Si prega di pulire l'ugello. Click per continuare."
#define MSG_WAITING_TEMP								"In attesa del raffreddamento della testina e del piatto"
#define MSG_FILAMENT_CLEAN								"Il colore e' nitido?"
#define MSG_UNLOADING_FILAMENT							"Rilasc. filamento"
#define MSG_PAPER										"Porre un foglio sotto l'ugello durante la calibrazione dei primi 4 punti. In caso l'ugello muova il foglio spegnere prontamente la stampante."

#define MSG_FINISHING_MOVEMENTS							"Arresto in corso"
#define MSG_PRINT_PAUSED								"Stampa in pausa"
#define MSG_RESUMING_PRINT								"Stampa in ripresa"
#define MSG_PID_EXTRUDER								"Calibrazione PID"
#define MSG_SET_TEMPERATURE								"Imposta temperatura"
#define MSG_PID_FINISHED								"Cal. PID completa"
#define MSG_PID_RUNNING									"Cal. PID"

#define MSG_CALIBRATE_PINDA								"Calibrare"
#define MSG_CALIBRATION_PINDA_MENU						"Taratura temp."
#define MSG_PINDA_NOT_CALIBRATED						"Taratura della temperatura non ancora eseguita"
#define MSG_PINDA_PREHEAT								"Riscald. PINDA"
#define MSG_TEMP_CALIBRATION							"Cal. temp.          "
#define MSG_TEMP_CALIBRATION_DONE						"Taratura temperatura terminata. Fare click per continuare."
#define MSG_TEMP_CALIBRATION_ON							"Cal. temp. [ON]"
#define MSG_TEMP_CALIBRATION_OFF						"Cal. temp. [OFF]"

#define MSG_LOAD_ALL									"Caricare tutti"
#define MSG_LOAD_FILAMENT_1								"Caricare fil. 1"
#define MSG_LOAD_FILAMENT_2								"Caricare fil. 2"
#define MSG_LOAD_FILAMENT_3								"Caricare fil. 3"
#define MSG_LOAD_FILAMENT_4								"Caricare fil. 4"
#define MSG_UNLOAD_FILAMENT_1							"Rilasciare fil. 1"
#define MSG_UNLOAD_FILAMENT_2							"Rilasciare fil. 1"
#define MSG_UNLOAD_FILAMENT_3							"Rilasciare fil. 1"
#define MSG_UNLOAD_FILAMENT_4							"Rilasciare fil. 1"
#define MSG_UNLOAD_ALL									"Rilasciare tutti"
#define MSG_PREPARE_FILAMENT							"Preparare filamento"
#define MSG_ALL											"Tutti"
#define MSG_USED										"Usati nella stampa"
#define MSG_CURRENT										"Attuale"
#define MSG_CHOOSE_EXTRUDER								"Seleziona estrusore:"
#define MSG_EXTRUDER									"Estrusore"
#define MSG_EXTRUDER_1									"Estrusore 1"
#define MSG_EXTRUDER_2									"Estrusore 2"
#define MSG_EXTRUDER_3									"Estrusore 3"
#define MSG_EXTRUDER_4									"Estrusore 4"
#define MSG_DATE							"Data"
#define MSG_XYZ_DETAILS						"XYZ Cal. dettagli"
#define	MSG_Y_DISTANCE_FROM_MIN				"Distanza Y da min:"
#define	MSG_LEFT							"Sinistra:"
#define MSG_CENTER							"Centro:"
#define MSG_RIGHT							"Destra:"
#define MSG_MEASURED_SKEW					"Incl. misurata:"
#define MSG_SLIGHT_SKEW						"Incl. leggera:"
#define MSG_SEVERE_SKEW						"Inc. rilevante:"
#define MSG_SORT_TIME						"Ordine: [Tempo]"
#define MSG_SORT_ALPHA						"Ordine:[Alfabeto]"
#define MSG_SORT_NONE						"Ordine: [Nessuno]"
#define MSG_WIZARD							"Wizard"
#define MSG_DEFAULT_SETTINGS_LOADED			"Settaggi predefiniti caricati"
#define MSG_SORTING							"Ordine dei file"
#define MSG_FILE_INCOMPLETE					"File incompleto. Continuare comunque?"
#define MSG_WIZARD_WELCOME					"Ciao, sono la tua stampante Original Prusa i3. Gradiresti aiuto attraverso il processo di configurazione?"
#define MSG_WIZARD_QUIT						"E possibile proseguire la guide Wizard in qualsiasi momento attraverso Calibrazione -> Wizard."
#define MSG_WIZARD_SELFTEST					"Anzitutto avviero il Self Test per controllare gli errori di assemblaggio piu comuni."
#define MSG_WIZARD_CALIBRATION_FAILED		"Per favore consulta il nostro manuale per risolvere il problema. Poi riprendi il Wizard dopo aver riavviato la stampante."
#define MSG_WIZARD_XYZ_CAL					"Adesso avviero una Calibrazione XYZ. Puo durare circa 12 min."
#define MSG_WIZARD_FILAMENT_LOADED			"Il filamento e stato caricato?"
#define MSG_WIZARD_Z_CAL					"Adesso avviero una Calibrazione Z."
#define MSG_WIZARD_WILL_PREHEAT				"Adesso preriscaldero l'ugello per PLA."
#define MSG_WIZARD_V2_CAL					"Adesso tarero lo stacco fra ugello e superfice del piatto."
#define MSG_WIZARD_V2_CAL_2					"Adesso iniziero a stampare una linea e tu dovrai abbassare l'ugello poco per volta ruotando la manopola sino a raggiungere una altezza ottimale. Per favore dai uno sguardo all'immagine del nostro manuale, cap.Calibrazione."
#define MSG_V2_CALIBRATION					"Cal. primo layer."
#define MSG_WIZARD_DONE						"Ben fatto. Buona stampa!"
#define MSG_WIZARD_LOAD_FILAMENT			"Per favore inserisci il filamento di PLA nell'estrusore, poi premi la manopola per caricare."
#define MSG_WIZARD_RERUN					"Se avvi il Wizard perderai la calibrazione preesistente e dovrai ricominciare dall'inizio. Continuare?"
#define MSG_WIZARD_REPEAT_V2_CAL			"Desideri ripetere l'ultimo passaggio per migliorare la distanza fra ugello e piatto?"
#define MSG_WIZARD_CLEAN_HEATBED			"Per favore pulisci il piatto, poi premi la manopola."
#define MSG_WIZARD_PLA_FILAMENT				"E questo un filamento di PLA?"
#define MSG_WIZARD_INSERT_CORRECT_FILAMENT	"Per favore carica filamento di PLA e riprendi il Wizard dopo aver riavviato la stampante."
#define MSG_PLA_FILAMENT_LOADED				"Il PLA e stato caricato?"
#define MSG_PLEASE_LOAD_PLA					"Per favore prima caricare filamento di PLA."
#define MSG_FILE_CNT						"Alcuni dei file non potranno essere organizzati. 100 e il n. max. di file che possono essere organizzati."
#define MSG_WIZARD_HEATING					"Sto preriscaldando l'ugello. Per favore attendi."
#define MSG_M117_V2_CALIBRATION				"M117 Cal. primo layer."
