#include <SPI.h>
#include <SD.h>

File collected_data;
bool print_data = false;
String test_string = "This is a test of saving to the SD card.";
double test_array[] = {2.2, 5.4, 7.6, 10.3, 9.5};
unsigned long time;

void setup() {
  // put your setup code here, to run once:

  initialize_SD_card();
  
  save_string(test_string);
  
  save_array(test_array, 5);
  
  read_file();
  
  //remove_data_file();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void initialize_SD_card()
  {
  Serial.begin(9600);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }
    
     Serial.print("Initializing SD card...");
  // On the Ethernet Shield, CS is pin 4. It's set as an output by default.
  // Note that even if it's not used as the CS pin, the hardware SS pin
  // (10 on most Arduino boards, 53 on the Mega) must be left as an output
  // or the SD library functions will not work.
  pinMode(10, OUTPUT);

  if (!SD.begin(10)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
 }
 
 void save_string(String string_to_save)
   {
     collected_data = SD.open("test.scv", FILE_WRITE);
     
     time = millis();
     
     collected_data.print(time);
     collected_data.println(" " + string_to_save);
     
     if (print_data)
     {
       Serial.print("This string is being printed to the SD card: ");
       Serial.println(string_to_save);
     }
     
     //SD.remove("test.txt");
     
     collected_data.close();
   }
   
 void save_array(double array_to_save[], int size_of_array)
   {
     collected_data = SD.open("test.txt", FILE_WRITE);
     
     time = millis();
     
     collected_data.print(time);
     collected_data.print(" ");

     int i;
       for (i=0; i < size_of_array; i++) {
         collected_data.print(array_to_save[i]);
         collected_data.print(" ");
         if (print_data){
           Serial.print("This array is being printed to the SD card: ");
           Serial.print(array_to_save[i]);
           Serial.print(" ");
         }
       }
        
       collected_data.println(" ");
       Serial.println();
     /*
     if (print_data)
     {
       int i;
       for (i=0; i < 4; i++) {
         Serial.print(array_to_save[i]);
         Serial.println(" ");
       }
     }
    */ 
     
     collected_data.close();
   }
   
   void read_file() 
   {
      collected_data = SD.open("test.txt");
    Serial.println("This is on the SD card:");

    // read from the file until there's nothing else in it:
    while (collected_data.available()) {
      Serial.write(collected_data.read());
    }
    // close the file:
    collected_data.close();
    
  }
  
  void remove_data_file()
  {
    SD.remove("test.txt");
  }
