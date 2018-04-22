/*----------------------------------------------------------------------------*\
|
|						     AGEIA PhysX Technology
|
|							     www.ageia.com
|
\*----------------------------------------------------------------------------*/

#pragma once
#ifndef _NX_DEBUGRENDER_H_
#define _NX_DEBUGRENDER_H_

class NxDebugRenderable;

class NxDebugRenderer
{
public:
	void renderData(const NxDebugRenderable& data) const;

private:
	static void renderBuffer(float* pVertList, float* pColorList, int type, int num);
};

#endif
//AGCOPYRIGHTBEGIN
///////////////////////////////////////////////////////////////////////////
// Copyright © 2005 AGEIA Technologies.
// All rights reserved. www.ageia.com
///////////////////////////////////////////////////////////////////////////
//AGCOPYRIGHTEND
