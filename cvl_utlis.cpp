//
//  cvl_utlis.cpp
//  raic
//
//  Created by Alex on 23/12/2018.
//  Copyright © 2018 Alex. All rights reserved.
//

#include "CVL_Utils.h"
#ifdef LOCAL_RUN
#include <fstream>
std::ofstream myfile;
#endif

void writeLog(const std::stringstream& ss){
#ifdef LOCAL_RUN
    if (!myfile.is_open())
        myfile.open ("example.txt");
    myfile << ss.str() <<std::endl;

#endif
}

void closeLog(){
#ifdef LOCAL_RUN
    if (myfile.is_open())
        myfile.close();
#endif
}
