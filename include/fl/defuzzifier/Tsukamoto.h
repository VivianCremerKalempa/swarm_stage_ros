/*   Copyright 2013 Juan Rada-Vilela

 Licensed under the Apache License, Version 2.0 (the "License");
 you may not use this file except in compliance with the License.
 You may obtain a copy of the License at

 http://www.apache.org/licenses/LICENSE-2.0

 Unless required by applicable law or agreed to in writing, software
 distributed under the License is distributed on an "AS IS" BASIS,
 WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 See the License for the specific language governing permissions and
 limitations under the License.
 */
/* 
 * File:   Tsukamoto.h
 * Author: jcrada
 *
 * Created on 13 December 2013, 4:24 PM
 */

#ifndef FL_TSUKAMOTO_H
#define	FL_TSUKAMOTO_H

#include "fuzzylite.h"

namespace fl {
    class Thresholded;
    
    class FL_EXPORT Tsukamoto {
    public:
        static scalar tsukamoto(const Thresholded* term, scalar minimum, scalar maximum);
    };

}

#endif	/* FL_TSUKAMOTO_H */

