/** @file HelloWorld.cpp
*
* @author	Marek Kopicki
*
*/

#include <Golem/Interop/HelloWorld/HelloWorld.h>
#include <memory>

using namespace golem;
using namespace golem::interop;

//------------------------------------------------------------------------------

std::shared_ptr<Interface> pInterface;

GOLEM_INTEROP_LIBRARY_DECLDIR void* InterfaceLoader(const char*) {
	if (!pInterface)
		pInterface.reset(new HelloWorld());
	return pInterface.get();
}

//------------------------------------------------------------------------------

void golem::interop::HelloWorld::run(const Map& interfaces) {
	printf("golem::interop::HelloWorld::run(): Hello World!\n");
}

//------------------------------------------------------------------------------
