/** @file Collision.cpp
 * 
 * Program for demonstrating and testing collision detection algorithms.
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/Sim/Universe.h>
#include <Golem/Plugin/Renderer.h>
#include <iostream>

using namespace golem;

//------------------------------------------------------------------------------

class Collision : public Object {
public:
	typedef shared_ptr<Collision> Ptr;
	
	/** Object description */
	class Desc : public Object::Desc {
	protected:
		/** Creates the object from the description. */
		CREATE_FROM_OBJECT_DESC_1(Collision, Object::Ptr, Scene&)

	public:
		/** Bounds descriptions */
		std::vector<golem::Bounds::Desc::Ptr> descriptions;
		/** Bounds descriptions indices */
		U32 n[2];
		/** Collision poses */
		Mat34 pose[2];
		/** Appearance */
		Appearance grayed, selected;
		/** Rotation and translation steps */
		Real rotation, translation;
		
		/** Constructs Collision description object */
		Desc() {
			Desc::setToDefault();
		}

		/** Sets the parameters to the default values */
		void setToDefault() {
			Object::Desc::setToDefault();

			// setup bounds
			descriptions.clear();
			
			BoundingSphere::Desc sphereDesc;
			sphereDesc.radius = Real(0.7);
			descriptions.push_back(sphereDesc.clone());
			
			BoundingBox::Desc boxDesc;
			boxDesc.dimensions.set(Real(0.5), Real(0.7), Real(1.5));
			descriptions.push_back(boxDesc.clone());
			boxDesc.dimensions.set(Real(0.1), Real(0.1), Real(0.3));
			descriptions.push_back(boxDesc.clone());
			
			BoundingCylinder::Desc cylinderDesc;
			cylinderDesc.length = Real(1.0);
			cylinderDesc.radius = Real(0.7);
			descriptions.push_back(cylinderDesc.clone());
			cylinderDesc.length = Real(0.7);
			cylinderDesc.radius = Real(0.2);
			descriptions.push_back(cylinderDesc.clone());
			
			n[0] = 1;
			n[1] = 2;

			pose[0].setId();
			pose[0].p.set(Real(+1.00), Real(0.0), Real(1.0));
			pose[1].setId();
			pose[1].p.set(Real(-1.00), Real(0.0), Real(1.0));
			
			selected.solidColour.set(192, 192, 0, 75); // set yellow
			selected.wireColour.set(127, 127, 127, 75); // set grey

			rotation = Real(0.1);
			translation = Real(0.01);
		}

		/** Checks if the description is valid. */
		bool isValid() const {
			if (!Object::Desc::isValid())
				return false;
			
			if (descriptions.empty())
				return false;
			if (n[0] >= descriptions.size() || n[1] >= descriptions.size())
				return false;
			if (!pose[0].isFinite() || !pose[1].isFinite())
				return false;
			if (!grayed.isValid() || !selected.isValid())
				return false;

			return true;
		}
	};

protected:
	/** Bounds descriptions */
	std::vector<golem::Bounds::Desc::Ptr> descriptions;
	/** Bounds descriptions indices */
	U32 n[2];
	/** Collision poses */
	Mat34 pose[2];
	/** Appearance */
	Appearance grayed, selected;
	/** Rotation and translation steps */
	Real rotation, translation;
	
	/** Bounds actors */
	Actor *actors[2];
	/** Selected bounds */
	U32 index;

	/** Generator of pseudo random numbers */
	Rand rand;
	
	int mouseButton;
	int mouseX;
	int mouseY;

	U32 getSelected() {
		return index;
	}
	U32 getGrayed() {
		return (index + 1)%2;
	}
	void next() {
		index = (index + 1)%2;
	}
	
	void mouseHandler(int button, int state, int x, int y) {
		mouseButton = button;
		mouseX = x;
		mouseY = y;

		if (button == 3 || button == 4) {
			Mat34 pose = actors[getSelected()]->getPose();
			
			Mat33 rotZ;
			rotZ.rotZ((REAL_PI*(button == 3 ? -1.0 : +1.0)*50.0/180.0)*rotation);
			pose.R.multiply(rotZ, pose.R);
			
			actors[getSelected()]->setPose(pose);

			collision();
		}
	}

	void motionHandler(int x, int y) {
		if (mouseButton == 2) {
			int dx = mouseX - x;
			int dy = mouseY - y;
			
			Mat34 pose = actors[getSelected()]->getPose();
			
			Mat33 rotX;
			rotX.rotX((REAL_PI*dx/180.0)*rotation);
			pose.R.multiply(rotX, pose.R);
			
			Mat33 rotY;
			rotY.rotY((REAL_PI*dy/180.0)*rotation);
			pose.R.multiply(rotY, pose.R);
			
			actors[getSelected()]->setPose(pose);

			mouseX = x;
			mouseY = y;		

			collision();
		}
	}

	/** Keyboard handler. */
	virtual void keyboardHandler(int key, int x, int y) {
		Mat34 newPose = actors[getSelected()]->getPose();
		Vec3 axis;
		axis.subtract(pose[getGrayed()].p, pose[getSelected()].p);
		axis.normalise();

		switch (key) {
		case '1':
			newPose.p.multiplyAdd(+translation, axis, newPose.p);
			break;
		case '2':
			newPose.p.multiplyAdd(-translation, axis, newPose.p);
			break;
		case 13:// enter
			n[getSelected()] = (n[getSelected()] + 1)%descriptions.size();
			(void)createActor(getSelected());
			return;
		case 9:// tab
			next();
			actors[getGrayed()]->setAppearance(grayed);
			actors[getSelected()]->setAppearance(selected);
			return;
		}

		actors[getSelected()]->setPose(newPose);

		collision();
	}
	
	/** Check for collisions. */
	void collision() {
		const Bounds::Ptr pBounds0 = actors[0]->getGlobalBoundsSeq()->front();
		const Bounds::Ptr pBounds1 = actors[1]->getGlobalBoundsSeq()->front();

		const char* str[2] = {"NO", "YES"};
		bool collision;
		
#ifdef _BOUNDS_PERFMON
		Bounds::resetLog();
#endif
		collision = pBounds0->intersect(*pBounds1);
#ifdef _BOUNDS_PERFMON
		context.info("collision0: %s, {iter = %d}\n", str[collision], Bounds::collisionConvexIter);
#else
		context.info("collision: %s\n", str[collision]);
#endif
	}

	/** Renders the Collision. */
	virtual void render() const {
	}

	/** Creates Actors. */
	virtual bool createActor(U32 index, bool bSelected = true) {
		Actor *&actor0 = actors[index];
		const golem::Bounds::Desc::Ptr &desc0 = descriptions[n[index]];
		const Mat34 pose0 = pose[index];

		Actor::Desc::Ptr actorDesc = scene.createActorDesc();
		actorDesc->boundsDescSeq.push_back(desc0);
		actorDesc->pose = pose0;
		actorDesc->kinematic = true;
		
		if (actor0 != NULL)
			scene.releaseObject(*actor0);	

		actor0 = dynamic_cast<Actor*>(scene.createObject(*actorDesc)); // throws
		
		actor0->setAppearance(bSelected ? selected : grayed);

		return true;
	}
	
	/** Creates the Collision from the Collision description. */
	void create(const Desc& desc) {
		descriptions = desc.descriptions;
		n[0] = desc.n[0];
		n[1] = desc.n[1];
		pose[0] = desc.pose[0];
		pose[1] = desc.pose[1];
		grayed = desc.grayed;
		selected = desc.selected;
		rotation = desc.rotation;
		translation = desc.translation;

		Object::create(desc); // throws
		createActor(getSelected()); // throws
		createActor(getGrayed(), false); // throws
	}

	/** Objects can be constructed only in the Scene context. */
	Collision(Scene &scene) :
		Object(scene), rand(context.getRandSeed())
	{
		actors[0] = actors[1] = NULL;
		index = 0;
	}
};

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	try {
		// Create program context
		golem::Context::Desc contextDesc;
		golem::Context::Ptr context = contextDesc.create();

		// Do not display LEVEL_DEBUG messages (only with level at least LEVEL_INFO)
		//context->getLogger()->setMsgFilter(MessageFilter::Ptr(new LevelFilter<Message>(Message::LEVEL_INFO)));

		//-----------------------------------------------------------------------------

		// Create Universe
		Universe::Desc universeDesc;
		universeDesc.name = "Golem (Collision)";
		universeDesc.argc = argc;
		universeDesc.argv = argv;
		Universe::Ptr pUniverse = universeDesc.create(*context);

		// Create scene
		Scene::Desc sceneDesc;
		sceneDesc.name = "Collision detection demo";
		Scene *pScene = pUniverse->createScene(sceneDesc);

		// Create Collision object
		context->info("Initialising collision object...\n");
		Collision::Desc collisionDesc;
		pScene->createObject(collisionDesc);
		
		// Launch universe
		context->info("Launching Universe...\n");
		pUniverse->launch();

		while (!pUniverse->interrupted())
			Sleep::msleep(SecToMSec(0.01));

		context->info("Good bye!\n");
	}
	catch (const Message& msg) {
		std::cerr << msg.what() << std::endl;
	}
	catch (const std::exception &ex) {
		std::cerr << Message(Message::LEVEL_CRIT, "C++ exception: %s", ex.what()).what() << std::endl;
	}

	return 0;
}
