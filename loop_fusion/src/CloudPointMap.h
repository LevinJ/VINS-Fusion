/*
 * CloudPointMap.h
 *
 *  Created on: Apr 16, 2020
 *      Author: levin
 */

#ifndef VINS_FUSION_LOOP_FUSION_CLOUDPOINTMAP_H_
#define VINS_FUSION_LOOP_FUSION_CLOUDPOINTMAP_H_

#include <list>
#include "keyframe.h"



class CloudPointMap {
public:
	CloudPointMap();
	virtual ~CloudPointMap();

	void saveMap(std::list<KeyFrame*> &keyframelist);
};

#endif /* VINS_FUSION_LOOP_FUSION_CLOUDPOINTMAP_H_ */
