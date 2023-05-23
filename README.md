# Lab3-Rcom
Trabalho Laboratorial 3 de Redes de Computadores FEUP

Realizado por Gonçalo Coelho (up202005368) e Marco Moreira (up202004135)

O ficheiro linklayer.c contém a implementação de todas as metas a atingir. 
Em send_frame e receive_frame foi atingida a meta de troca de carateres entre Tx e Rx e a criação da máquina de estados para receber a trama;
Em llopen foi implementada a troca da trama SET e trama UA e os mecanismos de temporização e retransmissão;
Em llwrite e llread foi atingida a meta de troca das tramas I, RR0 e RR1, REJ0 e REJ1. Foi também implementado o mecanismo stuffling, no emissor, e de-stufffling, no recetor, juntamente com a verificação do BCC2;
Em llclose, ocorre a troca das tramas DISC e UA, de modo a fechar a ligação.
