//
//  CVL_Utils.h
//  raic
//
//  Created by Alex on 18/12/2018.
//  Copyright © 2018 Alex. All rights reserved.
//

#ifndef CVL_Utils_h
#define CVL_Utils_h

#include <iostream>
#include <sstream>
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

#endif /* CVL_Utils_h */
