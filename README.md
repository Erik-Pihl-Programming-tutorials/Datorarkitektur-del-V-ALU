# Datorarkitektur (del V) - ALU
Femte delen i en serie som behandlar CPU-konstruktion i mjukvara. 
En simulerad processor som baseras på mikrodator ATmega328P implementeras i terminalmiljö,
där klockpulser, in- och utdata implementeras via inmatning samt utskrift från terminalen.

Denna del behandlar ALU:n, både sett till syfte, arbetssätt samt implementering för 
aritmetiska operationer addition och subtraktion samt logiska operationer OR, AND och XOR.
Statusbitar SNZVC is CPU:ns statusregister uppdateras utefter beräknat resultat.

Tidigare nämnda aritmetiska samt logiska operationer implementeras tillsammans med 
villkorliga hoppinstruktioner BREQ, BRNE, BRGE, BRGT, BRLE samt BRLT, som alla använder
sig av en eller flera statusbitar satta av ALU:n.

Även ytterligare instruktioner CLR, LSL samt LSR implementeras.
  
I efterföljande del ska interrupt-implementering läggas till.

Filen "alu.png" utgör en bild som demonstrerar ALU:ns arbetssätt visuellt samt via text.

Filen "alu_emulator.zip" utgör en ALU-emulator, som kan användas för att testa ALU:ns funktion
från en terminal. Operation samt operander kan matas in från terminalen, följt av att
resultatet skrivs ut både decimalt och binärt, tillsammans med statusbitar SNZVC.
Fem exempelfall skrivas ut vid start, som demonstrerar när de olika statusbitarna ettställs.

Filen "embedded_computer_system_part5_start_code.zip" innehåller startkoden 
(utan ALU-implementering, men med rikligt med kommentarer för filen alu.h).

Samtliga .c- och .h-filer utgörs av processorn med färdig ALU samt att programkoden har 
uppdaterats för att testa ALU:n via bitvis AND samt bitvis OR.

Filen "led_blink.asm" demonstrerar den assemblerkod som placeras i programminnet för 
att testa ALU:n, skrivet i AVR assembler.

Se video tutorial här:
https://youtu.be/nqKvC0gGxjI