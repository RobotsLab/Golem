/** @file Main.cpp
 * 
 * @author	Marek Kopicki
 *
 * @copyright  Copyright (C) 2015 Marek Kopicki, University of Birmingham, UK
 *
 * @license  This file copy is licensed to you under the terms described in
 *           the License.txt file included in this distribution.
 *
 */

#include <Golem/App/Manager.h>

//------------------------------------------------------------------------------

int main(int argc, char *argv[]) {
	return golem::Manager::Desc().main(argc, argv);
}
