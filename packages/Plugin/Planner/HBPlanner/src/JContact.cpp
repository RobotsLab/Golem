/** @file JContact.cpp
 * 
 * @author	Claudio Zito (The University Of Birmingham)
 *
 * @version 1.0
 *
 */
#include <Golem/HBPlanner/JContact.h>
#include <boost/lexical_cast.hpp>
#include <iomanip>

//------------------------------------------------------------------------------

using namespace golem;

//------------------------------------------------------------------------------

const std::string FTGuard::ChainName[] = {
	"UNKNOWN",
	"THUMB",
	"INDEX",
	"MIDDLE",
	"RING",
	"PINKY",
};

const char* FTGuard::ModeName[] = {
	"Disable",
	"Enable",
	"Contact",
};

const char* FTGuard::FaceName[] = {
	"UNKNOWN",
	"FRONT",
	"RIGHT",
	"LEFT",
	"BACK",
	"TIP",
	"TOP",
};


//------------------------------------------------------------------------------

FTGuard* FTGuard::Desc::create() const {
	FTGuard* guard(new FTGuard());
	guard->create(*this);
	return guard;
}

//------------------------------------------------------------------------------

FTGuard::FTGuard() {
}

void FTGuard::str(golem::Context& context) const {
	context.write("FTGuard[%s]: mode=%s, face=%s, wrench=[%f %f %f %f %f %f]\n", 
		ChainName[this->chain], ModeName[this->mode], this->faces.empty() ? FaceName[0] : FaceName[this->faces[0]],
		wrench.getV().x, wrench.getV().y, wrench.getV().z, wrench.getW().x, wrench.getW().y, wrench.getW().z);
//	std::string ss;
////	ss = "FTGuard[" + ChainName[this->chain] + "]: mode = " + ModeName[mode];// << " measured_force = " << strForces().c_str();
//	return ss;
}

std::string FTGuard::strForces() const {
	//	std::string ss = getGuardName(getHandChain()) + " joint=" + boost::lexical_cast<std::string>(getHandJoint()) + " measured_force=" + boost::lexical_cast<std::string>(force) + (type == FTGUARD_ABS ? " |> " : type == FTGUARD_LESSTHAN ? " < " : " > ") + boost::lexical_cast<std::string>(threshold);
	std::stringstream ss;
	ss << std::fixed << std::setprecision(3) << "[" + boost::lexical_cast<std::string>(wrench.getV().x) << " " << boost::lexical_cast<std::string>(wrench.getV().y) << " " << boost::lexical_cast<std::string>(wrench.getV().y) << " " <<
		boost::lexical_cast<std::string>(wrench.getW().x) << " " << boost::lexical_cast<std::string>(wrench.getW().y) << " " << boost::lexical_cast<std::string>(wrench.getW().z) + "]";
	//+(type == FTGUARD_ABS ? " |> " : type == FTGUARD_LESSTHAN ? " < " : " > ") + "[" +
	//	boost::lexical_cast<std::string>(limits[0]) + " " + boost::lexical_cast<std::string>(limits[1]) + " " + boost::lexical_cast<std::string>(limits[2]) + " " + boost::lexical_cast<std::string>(limits[3]) + " " +
	//	boost::lexical_cast<std::string>(limits[4]) + " " + boost::lexical_cast<std::string>(limits[5]) + "]";
	return ss.str();
}


void FTGuard::create(const FTGuard::Desc& desc) {
	chain = desc.chain;
	jointIdx = desc.jointIdx;
	type = desc.type;
	mode = desc.mode;
	limits = desc.limits;
	faces.clear();
	//	printf("FTGuard(armJoints=%u, handChains=%u, fingerJoints=%u) chain=%u, chain joint=%u, joint=%u\n", armIdx, handChains, fingerJoints, (handIdx / fingerJoints) + 1, handIdx % fingerJoints, i);
}

bool FTGuard::checkContacts() {
	if (mode == Mode::DISABLE)
		return false;
	if (mode == Mode::INCONTACT)
		return true;

	faces.clear();
	Real force = REAL_ZERO;
	for (size_t i = 0; i < 3; ++i) {
		if ((Math::abs(wrench.getV()[i]) > limits[i]) || (Math::abs(wrench.getW()[i]) > limits[i + 3])) {
			mode = Mode::INCONTACT;
			const Real f = std::max<Real>(Math::abs(wrench.getV()[i]), Math::abs(wrench.getW()[i]));
			const Face face = (i == 0 && (wrench.getV()[i] < REAL_ZERO || Math::abs(wrench.getW()[i]) > limits[i + 3])) ? Face::RIGHT :
				(i == 0 && (wrench.getV()[i] > REAL_ZERO || Math::abs(wrench.getW()[i]) < limits[i + 3])) ? Face::LEFT :
				(i == 1 && (wrench.getV()[i] > REAL_ZERO || Math::abs(wrench.getW()[i]) > limits[i + 3])) ? Face::TIP :
				(i == 2 && (wrench.getV()[i] < REAL_ZERO || Math::abs(wrench.getW()[i]) > limits[i + 3])) ? Face::FRONT : Face::BACK;
			if (f > force) {
				faces.insert(faces.begin(), face);
				force = f;
			}
			else
				faces.push_back(face);
		}

		//if ((Math::abs(wrench.getV()[i]) > limits[i]) || (Math::abs(wrench.getW()[i]) > limits[i+3])) {
		//	mode = Mode::INCONTACT;
		//	const Real f = std::max<Real>(Math::abs(wrench.getV()[i]), Math::abs(wrench.getW()[i]));
		//	const Face face = i == 0 ? Face::TIP : (i == 1 && (wrench.getV()[i] < REAL_ZERO || Math::abs(wrench.getW()[i]) > limits[i + 3])) ? Face::LEFT :
		//		(i == 1 && (wrench.getV()[i] > REAL_ZERO || Math::abs(wrench.getW()[i]) > limits[i + 3])) ? Face::RIGHT :
		//		(i == 2 && (wrench.getV()[i] < REAL_ZERO || Math::abs(wrench.getW()[i]) > limits[i + 3])) ? Face::FRONT : Face::BACK;
		//	if (f > force) {
		//		faces.insert(faces.begin(), face);
		//		force = f;
		//	}
		//	else
		//		faces.push_back(face);
		//}
	}
	return false;
}

//void XMLData(FTGuard::Desc& val, XMLContext* xmlcontext, bool create) {
//	golem::XMLData("name", val.name, xmlcontext, create);
////	golem::XMLData("joint", val.joint, xmlcontext, create);
//	golem::XMLData("type", val.type, xmlcontext, create);
////	golem::XMLData("value", val.value, xmlcontext, create);
//}

