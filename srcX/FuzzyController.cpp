#include "ros/ros.h"
#include "Eigen/Dense"

//Fuzzylite library
#include "fl/Headers.h"

using namespace fl;

using Eigen::MatrixXd;
using namespace std;

class FuzzyController{

public:
	float * fuzzyPositionController(float angularDistance, float linearDistance)
	{
		fl::Engine* engine = new fl::Engine;
		engine->setName("FollowerPositionController");

		fl::InputVariable* inputVariable1 = new fl::InputVariable;
		inputVariable1->setEnabled(true);
		inputVariable1->setName("AngularDistance");
		inputVariable1->setRange(-7.000, 7.000);
		inputVariable1->addTerm(new fl::Trapezoid("VeryRight", -7.000, -7.000, -0.500, -0.350));
		inputVariable1->addTerm(new fl::Triangle("Right", -0.500, -0.300, -0.100));
		inputVariable1->addTerm(new fl::Triangle("Front", -0.250, 0.000, 0.250));
		inputVariable1->addTerm(new fl::Triangle("Left", 0.100, 0.300, 0.500));
		inputVariable1->addTerm(new fl::Trapezoid("VeryLeft", 0.350, 0.500, 7.000, 7.000));
		engine->addInputVariable(inputVariable1);

		fl::InputVariable* inputVariable2 = new fl::InputVariable;
		inputVariable2->setEnabled(true);
		inputVariable2->setName("LinearDistance");
		inputVariable2->setRange(0.000, 10000.000);
		inputVariable2->addTerm(new fl::Trapezoid("Near", 0.000, 0.000, 1.000, 2.000));
		inputVariable2->addTerm(new fl::Triangle("Medium", 1.500, 2.500, 3.500));
		inputVariable2->addTerm(new fl::Trapezoid("Far", 3.000, 4.000, 10000.000, 10000.000));
		engine->addInputVariable(inputVariable2);

		fl::OutputVariable* outputVariable1 = new fl::OutputVariable;
		outputVariable1->setEnabled(true);
		outputVariable1->setName("AngularVelocity");
		outputVariable1->setRange(-2.000, 2.000);
		outputVariable1->fuzzyOutput()->setAccumulation(new fl::Maximum);
		outputVariable1->setDefuzzifier(new fl::Centroid(200));
		outputVariable1->setDefaultValue(0.000);
		outputVariable1->setLockValidOutput(false);
		outputVariable1->setLockOutputRange(false);
		outputVariable1->addTerm(new fl::Trapezoid("TurnVeryRight", -2.000, -2.000, -1.600, -1.100));
		outputVariable1->addTerm(new fl::Triangle("TurnRight", -1.300, -0.800, -0.300));
		outputVariable1->addTerm(new fl::Triangle("Front", -0.500, 0.000, 0.500));
		outputVariable1->addTerm(new fl::Triangle("TurnLeft", 0.300, 0.800, 1.300));
		outputVariable1->addTerm(new fl::Trapezoid("TurnVeryLeft", 1.100, 1.600, 2.000, 2.000));
		engine->addOutputVariable(outputVariable1);

		fl::OutputVariable* outputVariable2 = new fl::OutputVariable;
		outputVariable2->setEnabled(true);
		outputVariable2->setName("LinearVelocity");
		outputVariable2->setRange(0.200, 2.400);
		outputVariable2->fuzzyOutput()->setAccumulation(new fl::Maximum);
		outputVariable2->setDefuzzifier(new fl::Centroid(200));
		outputVariable2->setDefaultValue(0.000);
		outputVariable2->setLockValidOutput(false);
		outputVariable2->setLockOutputRange(false);
		outputVariable2->addTerm(new fl::Trapezoid("Low", 0.200, 0.200, 0.700, 1.200));
		outputVariable2->addTerm(new fl::Triangle("Medium", 0.950, 1.450, 1.950));
		outputVariable2->addTerm(new fl::Trapezoid("High", 1.700, 2.200, 2.400, 2.400));
		engine->addOutputVariable(outputVariable2);

		fl::RuleBlock* ruleBlock = new fl::RuleBlock;
		ruleBlock->setEnabled(true);
		ruleBlock->setName("");
		ruleBlock->setConjunction(new fl::Minimum);
		ruleBlock->setDisjunction(new fl::Maximum);
		ruleBlock->setActivation(new fl::Minimum);
		ruleBlock->addRule(fl::Rule::parse("if AngularDistance is VeryRight and LinearDistance is Near then AngularVelocity is TurnVeryRight and LinearVelocity is Low", engine));
		ruleBlock->addRule(fl::Rule::parse("if AngularDistance is VeryRight and LinearDistance is Medium then AngularVelocity is TurnVeryRight and LinearVelocity is Medium", engine));
		ruleBlock->addRule(fl::Rule::parse("if AngularDistance is VeryRight and LinearDistance is Far then AngularVelocity is TurnVeryRight and LinearVelocity is High", engine));
		ruleBlock->addRule(fl::Rule::parse("if AngularDistance is Right and LinearDistance is Near then AngularVelocity is TurnRight and LinearVelocity is Low", engine));
		ruleBlock->addRule(fl::Rule::parse("if AngularDistance is Right and LinearDistance is Medium then AngularVelocity is TurnRight and LinearVelocity is Medium", engine));
		ruleBlock->addRule(fl::Rule::parse("if AngularDistance is Right and LinearDistance is Far then AngularVelocity is TurnRight and LinearVelocity is High", engine));
		ruleBlock->addRule(fl::Rule::parse("if AngularDistance is Front and LinearDistance is Near then AngularVelocity is Front and LinearVelocity is Low", engine));
		ruleBlock->addRule(fl::Rule::parse("if AngularDistance is Front and LinearDistance is Medium then AngularVelocity is Front and LinearVelocity is Medium", engine));
		ruleBlock->addRule(fl::Rule::parse("if AngularDistance is Front and LinearDistance is Far then AngularVelocity is Front and LinearVelocity is High", engine));
		ruleBlock->addRule(fl::Rule::parse("if AngularDistance is Left and LinearDistance is Near then AngularVelocity is TurnLeft and LinearVelocity is Low", engine));
		ruleBlock->addRule(fl::Rule::parse("if AngularDistance is Left and LinearDistance is Medium then AngularVelocity is TurnLeft and LinearVelocity is Medium", engine));
		ruleBlock->addRule(fl::Rule::parse("if AngularDistance is Left and LinearDistance is Far then AngularVelocity is TurnLeft and LinearVelocity is High", engine));
		ruleBlock->addRule(fl::Rule::parse("if AngularDistance is VeryLeft and LinearDistance is Near then AngularVelocity is TurnVeryLeft and LinearVelocity is Low", engine));
		ruleBlock->addRule(fl::Rule::parse("if AngularDistance is VeryLeft and LinearDistance is Medium then AngularVelocity is TurnVeryLeft and LinearVelocity is Medium", engine));
		ruleBlock->addRule(fl::Rule::parse("if AngularDistance is VeryLeft and LinearDistance is Far then AngularVelocity is TurnVeryLeft and LinearVelocity is High", engine));
		engine->addRuleBlock(ruleBlock);

		//Set inputs
		inputVariable1->setInputValue(angularDistance); 
		inputVariable2->setInputValue(linearDistance); 
		
		//Start fuzzy
		engine->process();
	
		float *result = new float[2];
		//Defuzzification
		result[0] = outputVariable1->defuzzify();
		result[1] = outputVariable2->defuzzify();

		return result;
	}

};

