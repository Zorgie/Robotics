/*
 * Enumerations.h
 *
 *  Created on: 01.12.2013
 *      Author: paul
 */

#ifndef ENUMERATIONS_H_
#define ENUMERATIONS_H_

#include <iostream>
using namespace std;



enum object{
	TIGER=0,ZEBRA,ELEPHANT,HIPPO,GIRAFFE,LION,								//ANIMALS

	POTATO,TOMATO,ONION,BROCOLI,PAPRIKA,CARROT,CORN,AVOCADO,PEPPER,			//VEGETABLES

	MELON,PEAR,BANANA,ORANGE,LEMON,											//FRUITS

	PLATE,																	//AND THE PLATE

	NUMBER_OF_OBJECTS
};


enum colors{
	BANANA_YELLOW,			//0
	POTATO_BROWN,			//1
	TOMATO_RED,				//2
	ONION_ORANGE,
	BROCOLI_GREEN,
	PAPRIKA_GREEN,			//5
	CARROT_ORANGE,
	CARROT_GREEN,
	CORN_YELLOW,
	LEMON_YELLOW,
	PEAR_GREEN,				//10
	ORANGE_ORANGE,
	MELON_GREEN,
	MELON_RED,
	AVOCADO_GREEN,
	//LION_YELLOW,
	PEPPER_RED,
	PLATE_RED,

	NR_OF_COLORS
};






#endif /* ENUMERATIONS_H_ */
