/** @file Defs.h
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */
 
#pragma once
#ifndef _GOLEM_PLUGIN_DEFS_H_
#define _GOLEM_PLUGIN_DEFS_H_

//------------------------------------------------------------------------------

#include <Golem/Math/Twist.h>
#include <Golem/Math/VecN.h>
#include <Golem/Math/TriangleMesh.h>
#include <Golem/Sys/Defs.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Sys/Stream.h>
#include <vector>
#include <map>
#include <set>
#include <string>

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

class Context;
class XMLContext;
//class MsgXMLParser;
//template <typename SEQ> void XMLDataSeq(SEQ &seq, const char* attr, XMLContext* context, bool create, const typename SEQ::value_type& dflt);

//------------------------------------------------------------------------------

typedef std::vector<bool> BoolSeq;
typedef std::vector<int> IntSeq;
typedef std::vector<golem::U8> U8Seq;
typedef std::vector<golem::I32> I32Seq;
typedef std::vector<golem::U32> U32Seq;
typedef std::vector<golem::Real> RealSeq;
typedef std::vector<golem::Vec3> Vec3Seq;
typedef std::vector<golem::Mat34> Mat34Seq;
typedef std::vector<golem::Twist> TwistSeq;
typedef std::vector<std::string> StringSeq;
typedef std::vector<golem::Triangle> TriangleSeq;

typedef std::set<golem::U32> U32Set;
typedef std::set<std::string> StringSet;

typedef std::map<std::string, golem::Mat34> Mat34Map;
typedef std::map<golem::U32, golem::U32> U32Map;

//------------------------------------------------------------------------------

/** Dynamic casting _Inp to _Out */
template <typename _Out, typename _Inp> inline _Out* is(_Inp* inp) {
	return dynamic_cast<_Out*>(inp);
}
/** Dynamic casting _Inp to _Out */
template <typename _Out, typename _Inp> inline const _Out* is(const _Inp* inp) {
	return dynamic_cast<const _Out*>(inp);
}
/** Dynamic casting _Inp to _Out (reference argument assumes map<whatever, pointer>) */
template <typename _Out, typename _Inp> inline _Out* is(_Inp& inp) {
	return dynamic_cast<_Out*>(&*inp->second);
}
/** Dynamic casting _Inp to _Out (reference argument assumes map<whatever, pointer>) */
template <typename _Out, typename _Inp> inline const _Out* is(const _Inp& inp) {
	return dynamic_cast<const _Out*>(&*inp->second);
}

/** Static casting _Inp to _Out (assumes _Out is derived from _Inp) */
template <typename _Out, typename _Inp> inline _Out* to(_Inp* inp) {
	return static_cast<_Out*>(inp);
}
/** Static casting _Inp to _Out (assumes _Out is derived from _Inp) */
template <typename _Out, typename _Inp> inline const _Out* to(const _Inp* inp) {
	return static_cast<const _Out*>(inp);
}
/** Static casting (assumes _Type::Ptr is defined) */
template <typename _Type> inline _Type* to(const typename _Type::Ptr& inp) {
	return static_cast<_Type*>(inp.get());
}
/** Static casting (assumes _Type::Map is defined) */
template <typename _Type> inline _Type* to(typename _Type::Map::iterator& inp) {
	return static_cast<_Type*>(&*inp->second);
}
/** Static casting (assumes _Type::Map is defined) */
template <typename _Type> inline _Type* to(const typename _Type::Map::const_iterator& inp) {
	return static_cast<_Type*>(&*inp->second);
}

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
//void XMLData(const std::string &attr, golem::U32 &val, golem::XMLContext* context, bool create = false);

/** Configuration and 3D pose/frame */
template <typename _Pose> class ConfigPose {
public:
	typedef std::vector<ConfigPose> Seq;
	typedef std::multimap<std::string, ConfigPose> Map;
	typedef std::pair<typename Map::const_iterator, typename Map::const_iterator> Range;

	/** Configuration */
	RealSeq c;
	/** Frame */
	_Pose w;

	/** Planner index */
	golem::U32 plannerIndex;

	/** Set all to default values */
	ConfigPose() {
		setToDefault();
	}
	ConfigPose(const RealSeq& c, golem::U32 plannerIndex = 0) : c(c), plannerIndex(plannerIndex) {
		w.setId();
	}
	ConfigPose(const _Pose& w, golem::U32 plannerIndex = 0) : w(w), plannerIndex(plannerIndex) {
		c.clear();
	}
	ConfigPose(const RealSeq& c, const _Pose& w, golem::U32 plannerIndex = 0) : c(c), w(w), plannerIndex(plannerIndex) {
	}

	/** Set to id transformations and reset cpos */
	void setToDefault() {
		c.clear();
		w.setId();
		plannerIndex = 0;
	}
	/** Assert that the description is valid. */
	void assertValid(const Assert::Context& ac) const {
		for (RealSeq::const_iterator i = c.begin(); i != c.end(); ++i)
			Assert::valid(golem::Math::isFinite(*i), ac, "c[]: infinite");
		Assert::valid(w.isValid(), ac, "w: invalid");
	}

	/** Reads/writes object from/to a given XML context */
	void xmlData(golem::XMLContext* context, bool create = false) const {
		if (!create)
			const_cast<ConfigPose*>(this)->setToDefault();

		golem::XMLDataSeq(const_cast<RealSeq&>(c), "c", context, create, golem::REAL_ZERO);
		try {
			XMLData(const_cast<_Pose&>(w), context, create);
		}
		catch (const golem::MsgXMLParser&) {
			if (create) throw;
		}
		try {
			XMLData("planner_index", const_cast<golem::U32&>(plannerIndex), context, create);
		}
		catch (const golem::MsgXMLParser&) {
			if (create) throw;
		}
	}
};

typedef golem::ConfigPose<golem::Mat34> ConfigMat34;

/** Reads/writes object from/to a given XML context */
void XMLData(ConfigMat34 &val, golem::XMLContext* xmlcontext, bool create = false);
/** Reads/writes object from/to a given XML context */
void XMLData(ConfigMat34::Map::value_type &val, golem::XMLContext* xmlcontext, bool create = false);

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
template <typename _Real, size_t _N> void XMLData(golem::_VecN<_Real, _N>& value, const std::string& attr, golem::XMLContext* xmlcontext, bool create = false) {
	for (size_t i = 0; i < value.size(); ++i)
		golem::XMLData(attr + std::to_string(i + 1), value[i], xmlcontext, create);
}

/** Reads/writes object from/to a given XML context */
void XMLData(golem::Mat34Map::value_type& val, golem::XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

/** Formats a string */
std::string makeString(const char* format, ...);

/** Extracts directory */
std::string getDir(const std::string& path);

/** Extracts name */
std::string getName(const std::string& path);

/** Extracts extension */
std::string getExt(const std::string& path);

/** Compares extension */
bool isExt(const std::string& path, const std::string& ext);

//------------------------------------------------------------------------------

/** Data time stamp. */
class TimeStamp {
public:
	typedef std::vector<TimeStamp> Seq;

	/** Time stamp */
	golem::SecTmReal timeStamp;

	/** Default initialisation */
	TimeStamp(golem::SecTmReal timeStamp = golem::SEC_TM_REAL_ZERO) : timeStamp(timeStamp) {}

	/** Set time stamp */
	void set(const TimeStamp& timeStamp) {
		this->timeStamp = timeStamp.timeStamp;
	}
	/** Set time stamp */
	void get(TimeStamp& timeStamp) const {
		timeStamp.timeStamp = this->timeStamp;
	}

	/** Reads/writes object from/to a given XML context */
	void xmlData(golem::XMLContext* context, bool create = false) const;
};

//------------------------------------------------------------------------------

/** Header */
class Header : public Serializable {
public:
	/** Version*/
	enum {
		/** Undefined */
		VERSION_UNDEF = 0,
	};
	/** Version*/
	union Version {
		U32 version;
		struct {
			U16 major;
			U16 minor;
		};
	};

	/** Id length */
	static const size_t ID_LENGTH = 1 << 6;// golem::numeric_const<U8>::MAX;

										   /** Version */
	virtual Version getVersion() const = 0;
	/** Id */
	virtual const std::string& getId() const = 0;

	/** Current version */
	Version getVersionCurrent() const;
	/** Valid */
	bool isValid() const;

	/** Loads data from the specified stream */
	virtual void load(const Stream &stream);
	/** Stores data to the specified stream */
	virtual void store(Stream &stream) const;

	/** Initialisation */
	Header();

protected:
	/** Version */
	Version version;
	/** Valid */
	bool valid;
};

//------------------------------------------------------------------------------

/** Process debug interface.
*/
class DebugProcess {
public:
	/** Debug reset */
	virtual void debugReset() const = 0;
	/** Debug string */
	virtual void debugString(std::string& str) const = 0;
};

//------------------------------------------------------------------------------

/** Exception: program exit */
class Exit {};

/** Exception: cancel operation */
struct Cancel: public std::runtime_error { Cancel(const char* reason) : std::runtime_error(reason) {} };

//------------------------------------------------------------------------------

/** Object creating function from the description. */
#define GOLEM_CREATE_FROM_OBJECT_DESC1(OBJECT, POINTER, PARAMETER) virtual POINTER create(PARAMETER parameter) const {\
	OBJECT *pObject = new OBJECT(parameter);\
	POINTER pointer(pObject);\
	pObject->create(*this);\
	return pointer;\
}

/** Object creating function from the description. */
#define GOLEM_CREATE_FROM_OBJECT_DESC2(OBJECT, POINTER, PARAMETER1, PARAMETER2) virtual POINTER create(PARAMETER1 parameter1, PARAMETER2 parameter2) const {\
	OBJECT *pObject = new OBJECT(parameter1, parameter2);\
	POINTER pointer(pObject);\
	pObject->create(*this);\
	return pointer;\
}

//------------------------------------------------------------------------------

};	// namespace

//------------------------------------------------------------------------------

#endif /*_GOLEM_PLUGIN_DEFS_H_*/
