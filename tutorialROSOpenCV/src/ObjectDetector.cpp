/*
 * ObjectDetector.cpp
 *
 *  Created on: 26.11.2013
 *      Author: paul
 */

#include "ObjectDetector.h"
#include <iostream>

void ObjectDetector::printObjectName(object o){

	switch(o){
	case TIGER: cout << "TIGER" << endl; break;
	case ZEBRA: cout << "ZEBRA" << endl; break;
	case ELEPHANT: cout << "ELEPHANT" << endl; break;
	case HIPPO: cout << "HIPPO" << endl; break;
	case GIRAFFE: cout << "GIRAFFE" << endl; break;
	case POTATO: cout << "POTATO" << endl; break;
	case TOMATO: cout << "TOMATO" << endl; break;
	case ONION: cout << "ONION" << endl; break;
	case BROCOLI: cout << "BROCOLI" << endl; break;
	case PAPRIKA: cout << "PAPRIKA" << endl; break;
	case CARROT: cout << "CARROT" << endl; break;
	case CORN: cout << "CORN" << endl; break;
	case AVOCADO: cout << "AVOCADO" << endl; break;
	case MELON: cout << "MELON" << endl; break;
	case PEAR: cout << "PEAR" << endl; break;
	case BANANA: cout << "BANANA" << endl; break;
	case ORANGE: cout << "ORANGE" << endl; break;
	case LION: 	 cout << "LION" << endl;break;
	case LEMON:  cout << "LEMON" << endl;break;
	}
}


ObjectDetector::ObjectDetector() {
	for(int i = 0;i < NUMBER_OF_OBJECTS;i++){
		objectProbabilities[i] = 0.0;
	}
	detectingPhase = CONTOURING;
}


ObjectDetector::~ObjectDetector() {
}

void ObjectDetector::updateObjectProbability(cv::Mat imgBGR){
	switch(detectingPhase){
		case CONTOURING:
			//not enough contour votes?
			if(contourChecker.getCurrentNumberOfVotes() < 50){
				contourChecker.updateComplexityEstimation(imgBGR);
			}
			else{
				if(contourChecker.isComplex()){
					std::cout << "COMPLEX OBJECT DETECTED => GO SURFING" << std::endl;
					detectingPhase = HIGH_CONTOUR_SURFING;
				}
				else{
					std::cout << "SIMPLE OBJECT DETECTED => CHECK FOR SHAPE AND COLOR" << std::endl;
					detectingPhase = SHAPING;
				}
			}

			break;
		case HIGH_CONTOUR_SURFING:
			if(surfChecker.getNrOfSurfsDone() < 10){
				surfChecker.performSurf(imgBGR);
			}
			else{
				double minThreshold = 0;

				double avgGiraffe = surfChecker.getAvgGiraffeMatches();
				double avgZebra   = surfChecker.getAvgZebraMatches();
				double avgTiger   = surfChecker.getAvgTigerMatches();
				double sum = avgGiraffe + avgZebra + avgTiger;

				//only update the probabilities if the surf results actually indicate that there is (at least) one of those objects detected
				if(avgGiraffe > minThreshold && avgZebra > minThreshold && avgTiger > minThreshold){
					objectProbabilities[TIGER]   = avgTiger/sum;
					objectProbabilities[ZEBRA]   = avgZebra/sum;
					objectProbabilities[GIRAFFE] = avgGiraffe/sum;
				}
				detectingPhase = DONE;
			}
			break;

		case SHAPING:
			if(shapeChecker.nrOfVotes < 50){
				shapeChecker.updateShapeEstimation(imgBGR);
			}
			else
			{
				if(shapeChecker.isCircle()){
					cout << "ITS A CIRCLE => CHECK COLOR" << endl;
					objectProbabilities[TOMATO]  += 0.2;
					objectProbabilities[ONION]   += 0.2;
					objectProbabilities[BROCOLI] += 0.2;
					objectProbabilities[PAPRIKA] += 0.2;
					objectProbabilities[ORANGE]  += 0.2;
					objectProbabilities[AVOCADO]  += 0.2;
					objectProbabilities[LEMON]   += 0.2;

					detectingPhase = COLORING;

				}
				else if(shapeChecker.isEllipse())
				{
					cout << "ITS AN ELLIPSE => CHECK COLOR" << endl;
					objectProbabilities[POTATO] += 0.2;
					objectProbabilities[CARROT] += 0.2;
					objectProbabilities[MELON]  += 0.2;
					objectProbabilities[BANANA] += 0.2;
					objectProbabilities[CORN]   += 0.2;
					objectProbabilities[PEAR]   += 0.2;

					detectingPhase = COLORING;
				}
				else
				{
					//either the elephant or the hippo, we should try to detect the elephant using surf (maybe also
					//the hippo, not sure) and take the one with the best results
					//I guess all the other cases should be caught in the above cases
					objectProbabilities[ELEPHANT] += 0.2;
					objectProbabilities[HIPPO] += 0.2;
					cout << "NO CONTOUR FOUND => ELEPHANT OR HIPPO or LION - HANDLE THAT LATER" << endl;
					detectingPhase = LOW_CONTOUR_SURFING;
				}


			}

			break;
		case COLORING:

			if(colorDetector.getNumberOfIterations() < 50){
				cout << "COLORING STEP " << colorDetector.getNumberOfIterations() << endl;
				colorDetector.updatePixelCount(imgBGR);
			}
			else
			{
				cout << "COLOR DETECTION DONE: FOUND COLOR OBJECTS: " << endl;
				vector<object> result = colorDetector.getProbableObjects();
				for(int i = 0;i < result.size();i++)
				{
					cout << "result[i]: " << result[i] << endl;
					printObjectName(result[i]);
					objectProbabilities[result[i]] += 0.2;
				}
				detectingPhase = DONE;
			}

			break;

		case LOW_CONTOUR_SURFING:
			cout << "STARTED LOW CONTOUR SURFING" << endl;
			if(surfCheckerSimpleContours.getNrOfSurfsDone() < 3){
				cout << "LOW CONTOUR SURFING SUCCESSFUL!"<< surfCheckerSimpleContours.getNrOfSurfsDone()<< endl;

				surfCheckerSimpleContours.performSurf(imgBGR);
			}
			else{
				double minThreshold = 0;

				double avgElephant = surfCheckerSimpleContours.getAvgElephantMatches();
				double avgHippo   = surfCheckerSimpleContours.getAvgHippoMatches();
				double sum = avgElephant + avgHippo;

				//only update the probabilities if the surf results actually indicate that there is (at least) one of those objects detected
				if(avgElephant > minThreshold && avgHippo > minThreshold){
						objectProbabilities[ELEPHANT]   = avgElephant/sum;
						objectProbabilities[HIPPO]   = avgHippo/sum;
				}
				detectingPhase = DONE;
			}
			break;

		case DONE:
			break;

		default:
			break;
	}
}

//just return the object with the highest probability
vector<object> ObjectDetector::detectObject(){
	object bestGuess = static_cast<object>(0);
	double highestProbability = objectProbabilities[0];
	for(int i = 0;i < NUMBER_OF_OBJECTS;i++){
		if(objectProbabilities[i] > highestProbability){
			highestProbability = objectProbabilities[i];
			bestGuess = static_cast<object>(i);;
		}
	}

	vector<object> result;

	if (highestProbability >  0.3){
		for(int i = 0;i < NUMBER_OF_OBJECTS;i++){
			if(objectProbabilities[i] == highestProbability){
				result.push_back(static_cast<object>(i));
				cout << "PROBABLILTY: " << highestProbability << endl;
			}
		}
	}
	return result;
}

bool ObjectDetector::isFinished(){
	return detectingPhase == DONE;
}

void ObjectDetector::reset(){

	//reset sub-detectors
	surfChecker.reset();
	contourChecker.reset();
	shapeChecker.reset();
	colorDetector.reset();
	surfCheckerSimpleContours.reset();

	//reset object itself
	for(int i = 0;i < NUMBER_OF_OBJECTS;i++){
		objectProbabilities[i] = 0.0;
	}
	detectingPhase = CONTOURING;
}
