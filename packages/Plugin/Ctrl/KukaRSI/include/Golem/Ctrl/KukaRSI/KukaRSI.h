/** @file KukaRSI.h
*
* KukaRSI controller
*
* @author	Marek Kopicki
* @author	Maxime Adjigble
*
* @copyright  Copyright (C) 2015 Marek Kopicki and Maxime Adjigble, University of Birmingham, UK
*
* @license  This file copy is licensed to you under the terms described in
*           the License.txt file included in this distribution.
*
*/

#pragma once
#ifndef _GOLEM_CTRL_KUKARSI_KUKARSI_H_
#define _GOLEM_CTRL_KUKARSI_KUKARSI_H_

//------------------------------------------------------------------------------

#include <Golem/Ctrl/SingleCtrl/SingleCtrl.h>
#include <Golem/Sys/Library.h>
#include <Golem/Sys/XMLParser.h>
#ifdef WIN32
#include <winsock2.h>
#include <WS2tcpip.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <errno.h>
#include <unistd.h>
#endif

//------------------------------------------------------------------------------

extern "C" {
	GOLEM_LIBRARY_DECLDIR void loadControllerDesc(void* pContext, void* pXMLContext, void* pControllerDesc);
};

//------------------------------------------------------------------------------

namespace golem {

//------------------------------------------------------------------------------

//                                                                 
//            a1       a2                a3                        
//            |       ^                 ^                          
//            |      /                 /                        a5 
//            |     /                 /                         ^  
//            v    /                 /                         /   
//             l2 /       l3        /                         /    
//            ***O*****************O                         /     
//            * /             l4  /*                        /      
//			  */                 / ****************<-------O********
//            /                 /         l5      a6 & a4 /   l6    
//           /* l1             /                         /          
//            *                                         /           
//            *^ Z                                     /            
//            *|                                                    
//            *|                                                    
//            *| S     Y                                            
//             O-------->                                           
//            /                                                    
//           /                                                     
//          /                                                      
//         v X                                                    
//                                                                
//                                                                
//                                                                
//                           KR180 R2500                     KR180 R2900        
//  [m] (l)                                                             
//  l1                       0.675                           0.675                        
//  l2                       0.35                            0.35                       
//  l3                       1.15                            1.35                       
//  l4                       0.041                           0.041                        
//  l5                       1.0                             1.2                      
//  l6                       0.215                           0.215                        
//                                                                
//  [m] (x/y/z)
//  a1  (0,0,0)             (0,0,0)                         (0,0,0)                           
//  a2  (0,l2,l1)           (0,0.35,0.675)                  (0,0.35,0.675)                           
//  a3  (0,l2+l3,l1)        (0,1.5,0.675)                   (0,1.7,0.675)                          
//  a4  (0,l2+l3+l5,l1-l4)  (0,2.5,0.634)                   (0,2.9,0.634)                          
//  a5  (0,l2+l3+l5,l1-l4)  (0,2.5,0.634)                   (0,2.9,0.634)
//  a6  (0,l2+l3+l5,l1-l4)  (0,2.5,0.634)                   (0,2.9,0.634)
//                                                                
//  [m] (nx/ny/nz)
//  a1                      (0,0,-1)                        (0,0,-1)                              
//  a2                      (-1,0,0)                        (-1,0,0)                    
//  a3                      (-1,0,0)                        (-1,0,0)                    
//  a4                      (0,-1,0)                        (0,-1,0)                     
//  a5                      (-1,0,0)                        (-1,0,0)                    
//  a6                      (0,-1,0)                        (0,-1,0)                     
//                                                                
//  [deg] (min/max/zero)
//  a1                      (-185,+185,0)                   (-185,+185,0)                                       
//  a2                      (-140,-5,-90)                   (-140,-5,-90)                         
//  a3                      (-120,+155,+90)                 (-120,+155,+90)                           
//  a4                      (-350,+350,0)                   (-350,+350,0)                         
//  a5                      (-125,+125,0)                   (-125,+125,0)
//  a6                      (-350,+350,0)                   (-350,+350,0)
//                                                                
//  [rad] (min/max/zero)
//  a1                      (-3.14159,+3.14159,0)           (-3.14159,+3.14159,0)                                       
//  a2                      (-2.44346,-0.087266,-1.570796)  (-2.44346,-0.087266,-1.570796)                         
//  a3                      (-2.09439,+2.70526,+1.570796)   (-2.09439,+2.70526,+1.570796)                           
//  a4                      (-3.14159,+3.14159,0)           (-3.14159,+3.14159,0)                         
//  a5                      (-2.18166,+2.18166,0)           (-2.18166,+2.18166,0)
//  a6                      (-3.14159,+3.14159,0)           (-3.14159,+3.14159,0)
//                                                                
//  [deg/sec] (vmin/vmax)
//  a1                      (-123,+123)                     (-105,+105)                                       
//  a2                      (-115,+115)                     (-107,+107)                         
//  a3                      (-120,+120)                     (-114,+114)                           
//  a4                      (-179,+179)                     (-179,+179)                         
//  a5                      (-172,+172)                     (-172,+172)
//  a6                      (-219,+219)                     (-219,+219)
//                                                                
//  [rad/sec] (vmin/vmax)
//  a1                      (-2.146755,+2.146755)           (-1.832596,+1.832596)                                       
//  a2                      (-2.007129,+2.007129)           (-1.867502,+1.867502)                         
//  a3                      (-2.09439,+2.09439)             (-1.989675,+1.989675)                           
//  a4                      (-3.124139,+3.124139)           (-3.124139,+3.124139)                         
//  a5                      (-3.001966,+3.001966)           (-3.001966,+3.001966)
//  a6                      (-3.822271,+3.822271)           (-3.822271,+3.822271)

//------------------------------------------------------------------------------

class GOLEM_LIBRARY_DECLDIR KukaRSI: public SingleCtrl {
public:
	/** Number of axes */
	static const U32 NUM_JOINTS = 6;

	/** KukaRSI drive mode */
	enum DriveMode {
		/** Joint velocity/correction */
		JOINT_CORRECTION,
		/** Cartesian velocity */
		CARTESIAN_CORRECTION,
	};

	/** KukaRSI description */
	class GOLEM_LIBRARY_DECLDIR Desc: public SingleCtrl::Desc {
	public:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(KukaRSI, Controller::Ptr, Context&)

		/** Local port */
		U16 port;

		/** Message */
		std::string message;
		/** type */
		std::string type;

		/** Kuka drive mode */
		DriveMode driveMode;

		/** Feedback gain */
		std::vector<Real> gain;

		Desc() {
			setToDefault();
		}

		virtual void setToDefault() {
			SingleCtrl::Desc::setToDefault();

			cycleDurationMaxDev = SecTmReal(5.0);
			simDeltaRecv = SecTmReal(0.01);
			simDeltaSend = SecTmReal(0.002);

			name = "KukaRSI controller";

			port = 49148;

			message = "message";
			type = "type";

			driveMode = JOINT_CORRECTION;

			gain.assign(NUM_JOINTS, Real(0.02));

			chains.clear();
		}

		virtual bool isValid() const {
			if (!SingleCtrl::Desc::isValid())
				return false;

			if (gain.size() < (size_t)NUM_JOINTS)
				return false;

			return true;
		}
	};

	/** Release resources */
	virtual ~KukaRSI();

protected:
	/** Description file */
	Desc desc;

	/** Golem controller state */
	State::Ptr state;
	/** Kuka drive mode */
	DriveMode driveMode;

	Configspace::Index joint;

	virtual void sysSync();
	virtual void sysRecv(State& state);
	virtual void sysSend(const State& prev, const State& next, bool bSendPrev, bool bSendNext);

	virtual void userCreate();
	void create(const Desc& desc);

	KukaRSI(Context& context);

private:
	template <typename _Type> _Type getTagVal(char *xml, const char *tagname, const char *closetag, _Type(*atov)(const char*), char **pos) {
		const size_t tagLength = strlen(tagname);
		char *start = strstr(xml, tagname);
		if (start == NULL) {
			*pos = NULL;
			return 0;
		}
		char *end = strstr(start + tagLength, closetag);
		if (end == NULL) {
			*pos = NULL;
			return 0;
		}
		*end = '\0';
		const _Type ret = atov(start + tagLength);
		*end = closetag[0];
		*pos = end + strlen(closetag);
		return ret;
	}

	struct WorkspaceVar {
		double x, y, z, a, b, c;

		WorkspaceVar() {
			x = y = z = a = b = c = numeric_const<double>::ZERO;
		}
	};
	struct AxisVar {
		union {
			struct {
				double a1, a2, a3, a4, a5, a6;
			};
			double a[NUM_JOINTS];
		};

		AxisVar() {
			for (U32 i = 0; i < NUM_JOINTS; ++i)
				a[i] = numeric_const<double>::ZERO;
		}
	};

	struct IncomingData {
		WorkspaceVar actualCartPosition;
		AxisVar actualAxisPosition;
		
		I64 ipoc;
	};
	struct OutgoingData {
		std::string message;
		std::string type;

		WorkspaceVar cartCorrection;
		AxisVar axisCorrection;

		DriveMode driveMode;
	};

	int recvSocket;
	int sendSocket;

	sockaddr_in serverAddr;
	sockaddr_in clientAddr;

	IncomingData currentState;
	OutgoingData currentCommand;

	static const char* outgoingXMLFormat;

	std::vector<char> inBuf;
	std::vector<char> outBuf;

	bool parseIncomingState(char *xml);
	bool fillOutgoingBuffer();
};

//------------------------------------------------------------------------------

/** Reads/writes object from/to a given XML context */
void XMLData(Joint::Desc::Ptr &val, XMLContext* context, bool create = false);
void XMLData(Chain::Desc::Ptr &val, XMLContext* context, bool create = false);
void XMLData(KukaRSI::Desc &val, XMLContext* context, bool create = false);

//------------------------------------------------------------------------------

};	// namespace

#endif /*_GOLEM_CTRL_KUKARSI_KUKARSI_H_*/
