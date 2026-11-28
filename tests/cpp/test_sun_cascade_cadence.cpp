// Unit tests for SunCascadeCadence::Planner, which decides when a sun cascade past the first keeps last frame's map.

#include "Features/LightLimitFix/SunCascadeCadence.h"

#include <catch2/catch_test_macros.hpp>

#include <cmath>
#include <cstdint>
#include <limits>

using SunCascadeCadence::Box;
using SunCascadeCadence::Planner;
using SunCascadeCadence::Rules;
using SunCascadeCadence::View;

namespace
{
	View TwoCascades()
	{
		View view;
		view.light = 0x1000;
		view.cell = 0x2000;
		view.count = 2;
		view.rotation = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
		view.frustum = { -1, 1, 1, -1, 5, 10000 };
		view.cameras = { 0x3000, 0x3100 };
		view.slots = { 2ull << 32 | 0, 2ull << 32 | 1 };
		view.lightAxis = { { { 0.3f, 0.2f, -0.9f }, { 0.3f, 0.2f, -0.9f } } };
		view.splitStarts = { 0, 2400 };
		view.splitEnds = { 2500, 10240 };
		view.shadowDistance = 10240;
		return view;
	}

	// A box far larger than any view slice in these tests.
	Box Covering()
	{
		Box box;
		box.rotation = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
		box.frustum = { -40000, 40000, 40000, -40000, -40000, 40000 };
		return box;
	}

	// One frame as the engine runs it: plan, then every cascade that did not keep is drawn.
	uint32_t Frame(Planner& a_planner, const Rules& a_rules, const View& a_view, const Box& a_box = Covering())
	{
		const auto keep = a_planner.Plan(a_rules, a_view);
		for (uint32_t i = 0; i < a_view.count; ++i)
			if (!(keep >> i & 1))
				a_planner.Rendered(i, a_view, a_box);
		return keep;
	}

	View ThreeCascades()
	{
		auto view = TwoCascades();
		view.count = 3;
		view.cameras = { 0x3000, 0x3100, 0x3200 };
		view.slots = { 2ull << 32 | 0, 2ull << 32 | 1, 2ull << 32 | 2 };
		view.lightAxis = { { view.lightAxis[0], view.lightAxis[0], view.lightAxis[0] } };
		view.splitStarts = { 0, 2900, 5900 };
		view.splitEnds = { 3000, 6000, 10240 };
		return view;
	}
}

TEST_CASE("The first cascade never keeps its map", "[SunCascadeCadence]")
{
	Planner planner;
	Rules rules;
	const auto view = TwoCascades();
	for (int i = 0; i < 20; ++i)
		CHECK((Frame(planner, rules, view) & 1u) == 0);
}

TEST_CASE("The far cascade draws once every interval while nothing changes", "[SunCascadeCadence]")
{
	Planner planner;
	Rules rules;
	const auto view = TwoCascades();
	Frame(planner, rules, view);  // learns the cascade count
	Frame(planner, rules, view);  // first draw of both
	int drawn = 0;
	for (int i = 0; i < 40; ++i)
		drawn += (Frame(planner, rules, view) & 2u) ? 0 : 1;
	CHECK(drawn == 10);
}

TEST_CASE("An interval of 1 redraws every frame", "[SunCascadeCadence]")
{
	Planner planner;
	Rules rules;
	rules.farInterval = 1;
	const auto view = TwoCascades();
	for (int i = 0; i < 10; ++i)
		CHECK(Frame(planner, rules, view) == 0);
}

TEST_CASE("Moving, turning, a zoom, a new cell, camera, slot or light, or a sun turn redraws a kept cascade at once", "[SunCascadeCadence]")
{
	Rules rules;
	const auto base = TwoCascades();
	auto moved = base;
	moved.eye.x += 100;
	auto turned = base;
	turned.rotation = { 0.99f, -0.14f, 0, 0.14f, 0.99f, 0, 0, 0, 1 };  // about 8 degrees
	auto cell = base;
	cell.cell = 0x2100;
	auto sun = base;
	sun.lightAxis[1] = { 0.32f, 0.2f, -0.9f };
	auto slot = base;
	slot.slots[1] = 2ull << 32 | 2;
	auto zoom = base;
	zoom.frustum = { -0.8f, 0.8f, 0.8f, -0.8f, 5, 10000 };
	auto camera = base;
	camera.cameras[1] = 0x3400;
	auto light = base;
	light.light = 0x1100;
	for (const auto* changed : { &moved, &turned, &cell, &sun, &slot, &zoom, &camera, &light }) {
		Planner planner;
		Frame(planner, rules, base);
		Frame(planner, rules, base);
		CHECK((Frame(planner, rules, base) & 2u) != 0);
		CHECK((Frame(planner, rules, *changed) & 2u) == 0);
	}
}

TEST_CASE("A small move keeps the far cascade", "[SunCascadeCadence]")
{
	Planner planner;
	Rules rules;
	auto view = TwoCascades();
	Frame(planner, rules, view);
	Frame(planner, rules, view);
	view.eye.x += 10;
	CHECK((Frame(planner, rules, view) & 2u) != 0);
}

TEST_CASE("A turn the kept map does not cover redraws even under the turn limit", "[SunCascadeCadence]")
{
	Rules rules;
	auto view = TwoCascades();
	view.frustum = { -1, 1, 1, -1, 5, 20000 };
	Box fitted;  // exactly around cascade 1's slice of the straight-ahead view
	fitted.rotation = { 1, 0, 0, 0, 1, 0, 0, 0, 1 };
	fitted.frustum = { -10240, 10240, 10240, -10240, 2400, 10240 };
	Planner planner;
	Frame(planner, rules, view, fitted);
	Frame(planner, rules, view, fitted);
	CHECK((Frame(planner, rules, view, fitted) & 2u) != 0);
	const float s = std::sin(0.01745f), c = std::cos(0.01745f);  // one degree of yaw
	view.rotation = { c, -s, 0, s, c, 0, 0, 0, 1 };
	CHECK(SunCascadeCadence::TurnCosine(TwoCascades().rotation, view.rotation) >= rules.minTurnCosine);
	CHECK((Frame(planner, rules, view, fitted) & 2u) == 0);
}

TEST_CASE("A cascade whose end moved outward redraws", "[SunCascadeCadence]")
{
	Planner planner;
	Rules rules;
	auto view = TwoCascades();
	Frame(planner, rules, view);
	Frame(planner, rules, view);
	view.shadowDistance = 20000;
	view.splitEnds[1] = 20000;
	CHECK((Frame(planner, rules, view) & 2u) == 0);
}

TEST_CASE("A change in the cascade count forgets every kept map", "[SunCascadeCadence]")
{
	Planner planner;
	Rules rules;
	auto view = TwoCascades();
	Frame(planner, rules, view);
	Frame(planner, rules, view);
	view.count = 3;
	CHECK(Frame(planner, rules, view) == 0);
}

TEST_CASE("Forget makes the next frame redraw", "[SunCascadeCadence]")
{
	Planner planner;
	Rules rules;
	const auto view = TwoCascades();
	Frame(planner, rules, view);
	Frame(planner, rules, view);
	planner.Forget(~0u);
	CHECK(Frame(planner, rules, view) == 0);
}

TEST_CASE("The far cascade waits exactly the slider's frames", "[SunCascadeCadence]")
{
	Rules rules;
	const auto view = TwoCascades();
	for (uint32_t interval = 2; interval <= 8; ++interval) {
		rules.farInterval = interval;
		CHECK(SunCascadeCadence::Interval(rules, view, 1) == interval);
	}
}

TEST_CASE("With three cascades only the last waits the slider's frames", "[SunCascadeCadence]")
{
	Planner planner;
	Rules rules;
	const auto view = ThreeCascades();
	CHECK(SunCascadeCadence::Interval(rules, view, 0) == 1);
	CHECK(SunCascadeCadence::Interval(rules, view, 1) == 1);
	CHECK(SunCascadeCadence::Interval(rules, view, 2) == 4);
	Frame(planner, rules, view);
	int drawn = 0;
	for (int i = 0; i < 40; ++i) {
		const auto keep = Frame(planner, rules, view);
		CHECK((keep & 3u) == 0);
		drawn += (keep & 4u) ? 0 : 1;
	}
	CHECK(drawn == 10);
}

TEST_CASE("A non-finite view keeps nothing", "[SunCascadeCadence]")
{
	Planner planner;
	Rules rules;
	auto view = TwoCascades();
	Frame(planner, rules, view);
	Frame(planner, rules, view);
	view.eye.x = std::numeric_limits<float>::quiet_NaN();
	CHECK(Frame(planner, rules, view) == 0);
}
