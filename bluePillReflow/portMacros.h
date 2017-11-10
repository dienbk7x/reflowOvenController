#ifndef PORT_MACROS_H
#define PORT_MACROS_H

#define pinAsInput(P) pinMode(P,INPUT)
#define pinAsInputPullUp(P) pinMode(P,INPUT_PULLUP)
#define pinAsOutput(P) pinMode(P,OUTPUT)
#define digitalLow(P) digitalWrite(P,LOW)
#define digitalHigh(P) digitalWrite(P,HIGH)
#define isHigh(P)(digitalRead(P)>0)
#define isLow(P)(digitalRead(P)==0)
#define digitalState(P)(digitalRead(P))


#endif // PORT_MACROS
