/** @file PosePlanner.cpp
 * 
 * @author	Claudio Zito
 *
 * @version 1.0
 *
 */

#include <Golem/Data/Belief/Belief.h>
#include <Golem/Sys/XMLData.h>
#include <Golem/Tools/Import.h>
#include <Golem/Tools/Menu.h>
#include <boost/algorithm/string.hpp>

//-----------------------------------------------------------------------------

using namespace golem;
using namespace golem::data;

//-----------------------------------------------------------------------------

GOLEM_LIBRARY_DECLDIR void* golemDescLoader() {
	// Create description
	return new HandlerBelief::Desc();
}

//------------------------------------------------------------------------------

golem::data::ItemBelief::ItemBelief(HandlerBelief& handler) :
Item(handler), handler(handler), dataFile(handler.file), /*belief(nullptr),*/ 
modelItem(), queryItem(), queryItemSim(), meanPoseOnly(false), showQueryDistribution(false)
{
	modelFrame.setToDefault();
	queryTransform.setToDefault();
	queryTransformSim.setToDefault();
	modelPoints.clear();
	queryPoints.clear();
	queryPointsSim.clear();

	poses.clear();
	hypotheses.clear();
}

golem::data::Item::Ptr golem::data::ItemBelief::clone() const {
	Item::Ptr item(handler.create());
	ItemBelief* itemBeliefState = to<ItemBelief>(item.get());
	// copy the model
	itemBeliefState->setModelPoints(modelItem, this->modelFrame, modelPoints);
	// copy the query
	itemBeliefState->setQueryPoints(queryItem, this->queryPoints);
	// copy the sim object
	itemBeliefState->setSimObject(this->queryItemSim, this->queryTransformSim, this->queryPointsSim);
	// done!
	return item;
}

void golem::data::ItemBelief::createRender() {
	handler.createRender(*this);
}

void golem::data::ItemBelief::customRender() {
	handler.customRender();
}


void golem::data::ItemBelief::load(const std::string& prefix, const XMLContext* xmlcontext) {
	// belief
	std::string beliefSuffix;
	XMLData("ext_suffix", beliefSuffix, const_cast<XMLContext*>(xmlcontext), false);
	if (beliefSuffix.length() > 0) {
		dataFile.load(prefix + beliefSuffix, [&](const std::string& path) {
			FileReadStream frs(path.c_str());
			frs.read(modelItem);
			frs.read(modelFrame);
			frs.read(queryItem);
			frs.read(queryTransform);
			frs.read(queryItemSim);
			frs.read(queryTransformSim);
			poses.clear();
			frs.read(poses, poses.end());
			hypotheses.clear();
			frs.read(hypotheses, hypotheses.end());
		});
	}
}

void golem::data::ItemBelief::save(const std::string& prefix, XMLContext* xmlcontext) const {
	// belief xml
	std::string beliefSuffix = !poses.empty() ? handler.beliefSuffix : "";
	XMLData("ext_suffix", beliefSuffix, xmlcontext, true);

	// belief binary
	if (beliefSuffix.length() > 0) {
		dataFile.save(prefix + beliefSuffix, [=](const std::string& path) {
			FileWriteStream fws(path.c_str());
			fws.write(modelItem);
			fws.write(modelFrame);
			fws.write(queryItem);
			fws.write(queryTransform);
			fws.write(queryItemSim);
			fws.write(queryTransformSim);
			fws.write(poses.begin(), poses.end());
			fws.write(hypotheses.begin(), hypotheses.end());
		});
	}
	else
		dataFile.remove();
}

//------------------------------------------------------------------------------

golem::Belief::Desc::Ptr golem::data::ItemBelief::getBeliefDesc() const {
	return handler.desc.pBeliefDescPtr;
}

void golem::data::ItemBelief::set(const Mat34 queryTransform, const RBPose::Sample::Seq& poses, const RBPose::Sample::Seq& hypotheses) {
	this->queryTransform = queryTransform;
	this->poses = poses;
	this->hypotheses = hypotheses;
	dataFile.setModified(true);
}

void golem::data::ItemBelief::setModelPoints(const std::string modelItem, const Mat34 modelFrame, const Cloud::PointSeq& points) {
	this->modelItem = modelItem;
	this->modelFrame = modelFrame;
	this->modelPoints = points;
	dataFile.setModified(true);
}

void golem::data::ItemBelief::setQueryPoints(const std::string queryItem, const Cloud::PointSeq& points) {
	this->queryItem = queryItem;
	this->queryPoints = points;
	dataFile.setModified(true);
}
//------------------------------------------------------------------------------

void golem::data::HandlerBelief::Appearance::load(Context& context, const XMLContext* xmlcontext) {
	golem::XMLData("show_frame", showFrame, const_cast<XMLContext*>(xmlcontext), false);
	golem::XMLData("show_points", showPoints, const_cast<XMLContext*>(xmlcontext), false);
	try {
		golem::XMLData("distrib_samples", samples, const_cast<XMLContext*>(xmlcontext), false);
	}
	catch (const Message& msg) {}
	appearance.xmlData(const_cast<XMLContext*>(xmlcontext), false);
}

//------------------------------------------------------------------------------

void golem::data::HandlerBelief::Desc::load(Context& context, const XMLContext* xmlcontext) {
	golem::data::Handler::Desc::load(context, xmlcontext);

	golem::XMLData("ext_suffix", beliefSuffix, const_cast<XMLContext*>(xmlcontext), false);
	XMLContext* pxmlcontext = xmlcontext->getContextFirst("belief", false);
	golem::XMLData(*pBeliefDescPtr.get(), const_cast<XMLContext*>(pxmlcontext), false);

	posesAppearance.load(context, xmlcontext->getContextFirst("poses_appearance", false));
	meanPoseAppearance.load(context, xmlcontext->getContextFirst("meanpose_appearance", false));
	hypothesisAppearance.load(context, xmlcontext->getContextFirst("hypothesis_appearance", false));
	simulatedAppearance.load(context, xmlcontext->getContextFirst("ground_truth_appearance", false));
}

golem::data::Handler::Ptr golem::data::HandlerBelief::Desc::create(Context &context) const {
	golem::data::Handler::Ptr handler(new HandlerBelief(context));
	to<golem::data::HandlerBelief>(handler.get())->create(*this);
	return handler;
}

//------------------------------------------------------------------------------

golem::data::HandlerBelief::HandlerBelief(Context &context) : Handler(context) {
}

void golem::data::HandlerBelief::create(const Desc& desc) {
	data::Handler::create(desc);

	this->desc = desc;
	beliefSuffix = desc.beliefSuffix;
}

//------------------------------------------------------------------------------

std::string golem::data::HandlerBelief::getFileExtBelief() {
	return std::string(".hbs");
}

golem::data::Item::Ptr golem::data::HandlerBelief::create() const {
	return data::Item::Ptr(new ItemBelief(*const_cast<HandlerBelief*>(this))); // safe: no modification here - only passing reference
}

//------------------------------------------------------------------------------

void golem::data::HandlerBelief::createRender(const ItemBelief& item) {
	if (!item.belief.get())
		return;

	{
		UI::CriticalSectionWrapper cs(getUICallback());
		renderer.reset();

		// draw ground truth
		if (item.showSimulated && desc.simulatedAppearance.showPoints) {
			desc.simulatedAppearance.appearance.drawPoints(item.queryPointsSim, renderer);
		}

		// draw hypothesis
		for (Hypothesis::Seq::const_iterator i = item.belief->getHypotheses().begin(); i != item.belief->getHypotheses().end(); ++i) {
			Appearance& appearance = i == item.belief->getHypotheses().begin() ? desc.meanPoseAppearance : desc.hypothesisAppearance;
			renderer.addAxes((*i)->toRBPoseSampleGF().toMat34(), appearance.appearance.frameSize);
			appearance.appearance.drawPoints((*i)->getCloud(), renderer);
			if (item.meanPoseOnly) // this parameter can be control by outside
				break;
		}

		// draw query distribution as set of particles (use to debug the belief update)
		if (item.showQueryDistribution && desc.posesAppearance.showPoints) {
			const Real max = item.belief->maxWeight(true);
			//1 / (2 * (sigma*sqrt(2 * pi)))*exp(-.5*((x - mu) / sigma). ^ 2);
			const Real sigma = 0.2, norm = 1 / (2 * (sigma*Math::sqrt(2 * REAL_PI)));
			for (auto i = 0; i < item.belief->getSamples().size(); ++i) {
				if (item.belief->getSamples().size() > 10 && i % 10 != 0) continue;
				const RBPose::Sample &j = item.belief->getSamples()[i];
				renderer.addAxes(j.toMat34() * item.modelFrame, desc.posesAppearance.appearance.frameSize*item.belief->normalise(j));
				desc.posesAppearance.appearance.colourOverride = true;
				const Real weight = item.belief->normalise(j) / max;
				const Real red = norm*Math::exp(-.5*Math::sqr((weight - 1) / sigma)) * 255;
				//context.write("red = %f, U8(red)=%f\n", norm*Math::exp(-.5*Math::sqr((weight - 1) / sigma)), U8(norm*Math::exp(-.5*Math::sqr((weight - 1) / sigma))));
				const Real green = norm*Math::exp(-.5*Math::sqr((weight - .5) / sigma)) * 255;
				const Real blue = norm*Math::exp(-.5*Math::sqr((weight - .0) / sigma)) * 255;
				///*j.weight*/Math::log2(1+pBelief->normalise(j)/max)*255;
				//context.debug("weight=%f, normalised=%f, red=%u, green=%u, blue=%u\n", pBelief->normalise(j), pBelief->normalise(j) / max, U8(red), U8(green), U8(blue));
				desc.posesAppearance.appearance.colour = weight > REAL_ZERO ? RGBA(U8(red), U8(green), U8(blue), 200) : RGBA::BLACK;
				Cloud::PointSeq m;
				Cloud::transform(j.toMat34(), item.modelPoints, m);
				//						if (weight > 0.7)
				desc.posesAppearance.appearance.drawPoints(m, renderer);
			}
		}
		// draw query distribution as set of (randomly sampled) frames
		if (item.showQueryDistribution && desc.posesAppearance.showFrame) {
			for (size_t i = 0; i < desc.posesAppearance.samples; ++i)
				renderer.addAxes(item.belief->sample().toMat34() * item.modelFrame, desc.posesAppearance.appearance.frameSize);
		}
	}
}

void golem::data::HandlerBelief::render() const {
	renderer.render();
}

void golem::data::HandlerBelief::customRender() {
	if (hasRenderBlock()) return;

	requestRender();
//	renderer.render();
}

void golem::data::HandlerBelief::customRender() const {
}

//------------------------------------------------------------------------------

void golem::data::HandlerBelief::mouseHandler(int button, int state, int x, int y) {
	if (hasRenderBlock()) return;
}

void golem::data::HandlerBelief::motionHandler(int x, int y) {
}

void golem::data::HandlerBelief::keyboardHandler(int key, int x, int y) {
	if (hasRenderBlock()) return;
}