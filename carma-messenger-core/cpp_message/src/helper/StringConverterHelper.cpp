/*
 * Copyright (C) 2018-2020 LEIDOS.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

/**
 * This class has three functionalities. It can convert a string type into
 * a byte array and set the byte array as a field based on input field parameter.
 * It will handle both a field with required length or a field with dynamic
 * length. It can also convert a long type timestamp into a byte array and
 * set the byte array as a filed based on the input field parameter. This class
 * will be used mainly by different Mobility helper class. Its third functionality
 * is to read all data between '[' and ']' from a byte[] and return as a String. 
 */
#include<iostream>
#include<string>
#include<limits.h>


class StringConverterHelper {
    public:
    std::string DYNAMIC_STRING_DEFAULT="[]";
    int TIMESTAMP_LENGTH=std::to_string(LONG_MAX).length();
    
    //Returns a string based on length of input string
    std::string setDynamicLengthString(std::string inputString, int maxLength){
        std::string tmp;
        if(inputString.length() <=maxLength && inputString.length()!=0){
            tmp=inputString;
        }
        else{
            tmp=DYNAMIC_STRING_DEFAULT;
        }
        return tmp;
    }

    std::string readDynamicLengthString(std::string input){
        std::string buffer;
        for(int i=0;i<input.length();i++){
            if(input[i]==0){
                break;
            }
            else{
                buffer.push_back(input[i]);
            }
        }
    }

}