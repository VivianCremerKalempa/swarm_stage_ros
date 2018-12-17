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
 * Headers.h
 *
 *  Created on: 30/11/2012
 *      Author: jcrada
 */

#ifndef FL_HEADERS_H
#define FL_HEADERS_H

#include "fuzzylite.h"

#include "Console.h"
#include "Engine.h"
#include "Exception.h"

#include "defuzzifier/Bisector.h"
#include "defuzzifier/Centroid.h"
#include "defuzzifier/Defuzzifier.h"
#include "defuzzifier/IntegralDefuzzifier.h"
#include "defuzzifier/SmallestOfMaximum.h"
#include "defuzzifier/LargestOfMaximum.h"
#include "defuzzifier/MeanOfMaximum.h"
#include "defuzzifier/WeightedAverage.h"
#include "defuzzifier/WeightedSum.h"

#include "factory/Factory.h"
#include "factory/FactoryManager.h"
#include "factory/DefuzzifierFactory.h"
#include "factory/HedgeFactory.h"
#include "factory/SNormFactory.h"
#include "factory/TNormFactory.h"
#include "factory/TermFactory.h"

#include "imex/CppExporter.h"
#include "imex/FclImporter.h"
#include "imex/FclExporter.h"
#include "imex/FisImporter.h"
#include "imex/FisExporter.h"
#include "imex/FldExporter.h"
#include "imex/FllImporter.h"
#include "imex/FllExporter.h"
#include "imex/JavaExporter.h"

#include "hedge/Any.h"
#include "hedge/Extremely.h"
#include "hedge/Hedge.h"
#include "hedge/Not.h"
#include "hedge/Seldom.h"
#include "hedge/Somewhat.h"
#include "hedge/Very.h"

#include "Operation.h"
#include "norm/Norm.h"
#include "norm/SNorm.h"
#include "norm/TNorm.h"

#include "norm/s/AlgebraicSum.h"
#include "norm/s/BoundedSum.h"
#include "norm/s/DrasticSum.h"
#include "norm/s/EinsteinSum.h"
#include "norm/s/HamacherSum.h"
#include "norm/s/Maximum.h"
#include "norm/s/NormalizedSum.h"

#include "norm/t/AlgebraicProduct.h"
#include "norm/t/BoundedDifference.h"
#include "norm/t/DrasticProduct.h"
#include "norm/t/EinsteinProduct.h"
#include "norm/t/HamacherProduct.h"
#include "norm/t/Minimum.h"

#include "rule/Antecedent.h"
#include "rule/Consequent.h"
#include "rule/Rule.h"
#include "rule/RuleBlock.h"
#include "rule/Expression.h"


#include "term/Accumulated.h"
#include "term/Bell.h"
#include "term/Constant.h"
#include "term/Discrete.h"
#include "term/Function.h"
#include "term/Gaussian.h"
#include "term/GaussianProduct.h"
#include "term/Linear.h"
#include "term/PiShape.h"
#include "term/Ramp.h"
#include "term/Rectangle.h"
#include "term/SShape.h"
#include "term/Sigmoid.h"
#include "term/SigmoidDifference.h"
#include "term/SigmoidProduct.h"
#include "term/Term.h"
#include "term/Thresholded.h"
#include "term/Trapezoid.h"
#include "term/Triangle.h"
#include "term/ZShape.h"

#include "variable/InputVariable.h"
#include "variable/OutputVariable.h"
#include "variable/Variable.h"


#endif /* FL_HEADERS_H */
