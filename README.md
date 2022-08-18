# Termoigrometro ST03

Termoigrometro RS485 basato su scheda H10
e sistema operativo RIOT

## Compilazione

Per creare una immagine firmware installare RIOT su un PC Linux
seguendo questa procedura:

* <https://www.acmesystems.it/st03-riot>

Quindi clonare il repository che contiene l'applicazione lore-st03:

	git clone https://github.com/tanzilli/lora3a-st03

e compilare


	cd lora3a-st03
	sudo make flash

## Programma di test RS485


Per poter testare le schede ST03 ho scrito questo programmino in C di 
test che interroga via RS485 uno o più nodo ST03

	test/polling.c

Il file test_st03.c è un programmino di test da compilare su Raspberry
per provare i nodi su bus RS485 tramite un adattatore USB/RS485

Per compilare il programma di test:

	gcc polling.c -o polling


## Links

* Maggiori informazioni su: <https://www.acmesystems.it/st03>

Acme Systems srl (c) 2022



