#pragma once

#include <array>
#include <cmath>
#include <cstdint>

namespace SunCascadeCadence
{
	inline constexpr uint32_t kMaxCascades = 3;

	/** @brief When a sun cascade past the first may keep last frame's map instead of redrawing. */
	struct Rules
	{
		uint32_t farInterval = 4;          ///< frames the last cascade may keep its map; 1 redraws every frame
		float maxMove = 64.0f;             ///< game units the view may move since the cascade drew
		float minTurnCosine = 0.99939f;    ///< cos(2 degrees) of the view's turn since then
		float minSunCosine = 0.99999962f;  ///< cos(0.05 degrees) of the cascade's light direction since then
	};

	struct Vec3
	{
		float x = 0, y = 0, z = 0;
	};

	/** @brief An orthographic cascade camera: its world position, world rotation (row-major) and left, right, top, bottom, near, far. */
	struct Box
	{
		Vec3 origin{};
		std::array<float, 9> rotation{};
		std::array<float, 6> frustum{};
	};

	/** @brief What a cascade's kept map depends on, read each frame before the sun gathers. */
	struct View
	{
		Vec3 eye{};
		std::array<float, 9> rotation{};  ///< the view camera's world rotation
		std::array<float, 6> frustum{};   ///< the view camera's left, right, top, bottom, near, far
		uintptr_t light = 0;
		uintptr_t cell = 0;  ///< interior cell, else worldspace
		uint32_t count = 0;
		float shadowDistance = 0;
		std::array<uintptr_t, kMaxCascades> cameras{};
		std::array<uint64_t, kMaxCascades> slots{};  ///< render target << 32 | slice
		std::array<Vec3, kMaxCascades> lightAxis{};  ///< each cascade camera's view axis
		std::array<float, kMaxCascades> splitStarts{};
		std::array<float, kMaxCascades> splitEnds{};
	};

	/** @brief Frames a cascade may wait: the slider's for the last cascade past the first, else 1. */
	[[nodiscard]] inline uint32_t Interval(const Rules& a_rules, const View& a_view, uint32_t a_cascade) noexcept
	{
		if (a_cascade == 0 || a_rules.farInterval <= 1)
			return 1;
		return a_cascade + 1 == a_view.count ? a_rules.farInterval : 1;
	}

	/** @brief The cosine of the angle between two row-major rotations; -1 when either is not finite. */
	[[nodiscard]] inline float TurnCosine(const std::array<float, 9>& a_from, const std::array<float, 9>& a_to) noexcept
	{
		float trace = 0;
		for (size_t i = 0; i < a_from.size(); ++i)
			trace += a_from[i] * a_to[i];
		return std::isfinite(trace) ? (trace - 1.0f) * 0.5f : -1.0f;
	}

	/** @brief True when two directions lie within the angle whose cosine is a_minCosine; false for a zero vector. */
	[[nodiscard]] inline bool AxisWithin(float a_minCosine, Vec3 a_from, Vec3 a_to) noexcept
	{
		const double dot = double(a_from.x) * a_to.x + double(a_from.y) * a_to.y + double(a_from.z) * a_to.z;
		const double lengths = std::sqrt((double(a_from.x) * a_from.x + double(a_from.y) * a_from.y + double(a_from.z) * a_from.z) *
										 (double(a_to.x) * a_to.x + double(a_to.y) * a_to.y + double(a_to.z) * a_to.z));
		return lengths > 0 && dot >= a_minCosine * lengths;
	}

	/** @brief True when the view's slice of a cascade, a_view.splitStarts to splitEnds, lies inside the box its map was drawn from. */
	[[nodiscard]] inline bool Covers(const View& a_view, uint32_t a_cascade, const Box& a_box) noexcept
	{
		constexpr float kTolerance = 1e-3f;  // of the box's own size, for the fit's rounding at its edges
		const auto& r = a_view.rotation;
		const auto& b = a_box.rotation;
		const auto& f = a_box.frustum;
		const float slack[3] = { (f[5] - f[4]) * kTolerance, (f[1] - f[0]) * kTolerance, (f[2] - f[3]) * kTolerance };
		for (const float depth : { a_view.splitStarts[a_cascade], a_view.splitEnds[a_cascade] }) {
			for (const float up : { a_view.frustum[3], a_view.frustum[2] }) {
				for (const float right : { a_view.frustum[0], a_view.frustum[1] }) {
					// a view camera looks along its first axis, up its second and right along its third
					const double p[3] = { a_view.eye.x + depth * (r[0] + up * r[1] + right * r[2]) - a_box.origin.x,
						a_view.eye.y + depth * (r[3] + up * r[4] + right * r[5]) - a_box.origin.y,
						a_view.eye.z + depth * (r[6] + up * r[7] + right * r[8]) - a_box.origin.z };
					double along[3];
					for (int k = 0; k < 3; ++k)
						along[k] = p[0] * b[k] + p[1] * b[3 + k] + p[2] * b[6 + k];
					if (!(along[0] >= f[4] - slack[0] && along[0] <= f[5] + slack[0] && along[2] >= f[0] - slack[1] && along[2] <= f[1] + slack[1] &&
							along[1] >= f[3] - slack[2] && along[1] <= f[2] + slack[2]))
						return false;
				}
			}
		}
		return true;
	}

	/** @brief Plans which cascades keep their map this frame and records the views their maps were drawn in. */
	class Planner
	{
	public:
		void Reset() noexcept
		{
			history = {};
			count = 0;
		}

		/** @brief Once per frame before the sun gathers; returns the cascades that keep their map. */
		[[nodiscard]] uint32_t Plan(const Rules& a_rules, const View& a_view) noexcept
		{
			++frame;
			if (a_view.count != count || a_view.count > kMaxCascades) {
				Reset();
				count = a_view.count <= kMaxCascades ? a_view.count : 0;
				return 0;
			}
			if (!Usable(a_view))
				return 0;
			uint32_t keep = 0;
			for (uint32_t i = 1; i < count; ++i) {
				const auto interval = Interval(a_rules, a_view, i);
				const auto& drawn = history[i];
				if (interval <= 1 || !drawn.valid || frame - drawn.frame >= interval || !Holds(a_rules, a_view, i))
					continue;
				keep |= 1u << i;
			}
			return keep;
		}

		/** @brief After a cascade's map was drawn this frame from a_view by the cascade camera a_box. */
		void Rendered(uint32_t a_cascade, const View& a_view, const Box& a_box) noexcept
		{
			if (a_cascade >= kMaxCascades)
				return;
			auto& h = history[a_cascade];
			h.valid = true;
			h.frame = frame;
			h.eye = a_view.eye;
			h.rotation = a_view.rotation;
			h.frustum = a_view.frustum;
			h.light = a_view.light;
			h.cell = a_view.cell;
			h.camera = a_view.cameras[a_cascade];
			h.slot = a_view.slots[a_cascade];
			h.lightAxis = a_view.lightAxis[a_cascade];
			h.splitEnd = a_view.splitEnds[a_cascade];
			h.shadowDistance = a_view.shadowDistance;
			h.box = a_box;
		}

		void Forget(uint32_t a_mask) noexcept
		{
			for (uint32_t i = 0; i < kMaxCascades; ++i)
				if (a_mask >> i & 1)
					history[i].valid = false;
		}

	private:
		struct Drawn
		{
			bool valid = false;
			uint64_t frame = 0;
			Vec3 eye{};
			std::array<float, 9> rotation{};
			std::array<float, 6> frustum{};
			uintptr_t light = 0, cell = 0, camera = 0;
			uint64_t slot = 0;
			Vec3 lightAxis{};
			float splitEnd = 0, shadowDistance = 0;
			Box box{};
		};

		static bool Finite(Vec3 a_v) noexcept { return std::isfinite(a_v.x) && std::isfinite(a_v.y) && std::isfinite(a_v.z); }

		static constexpr float kGrowth = 1.02f;  ///< a split or shadow distance this much farther than when drawn has grown

		static bool Grew(float a_drawn, float a_now) noexcept
		{
			return std::isfinite(a_drawn) && std::isfinite(a_now) && a_drawn > 0.0f && a_now > a_drawn * kGrowth;
		}

		bool Usable(const View& a_view) const noexcept
		{
			for (const float plane : a_view.frustum)
				if (!std::isfinite(plane))
					return false;
			for (uint32_t i = 0; i < count; ++i)
				if (!Finite(a_view.lightAxis[i]))
					return false;
			return Finite(a_view.eye);
		}

		bool Holds(const Rules& a_rules, const View& a_view, uint32_t a_cascade) const noexcept
		{
			const auto& h = history[a_cascade];
			if (h.light != a_view.light || h.camera != a_view.cameras[a_cascade] || h.slot != a_view.slots[a_cascade] || h.cell != a_view.cell)
				return false;
			if (!AxisWithin(a_rules.minSunCosine, h.lightAxis, a_view.lightAxis[a_cascade]))
				return false;
			for (size_t k = 0; k < h.frustum.size(); ++k)
				if (k != 4 && h.frustum[k] != a_view.frustum[k])  // the near plane moves with camera mods and changes nothing past cascade 0
					return false;
			if (Grew(h.splitEnd, a_view.splitEnds[a_cascade]) || (a_cascade + 1 == count && Grew(h.shadowDistance, a_view.shadowDistance)))
				return false;
			const double dx = double(h.eye.x) - a_view.eye.x, dy = double(h.eye.y) - a_view.eye.y, dz = double(h.eye.z) - a_view.eye.z;
			if (!(dx * dx + dy * dy + dz * dz <= double(a_rules.maxMove) * a_rules.maxMove))
				return false;
			return TurnCosine(h.rotation, a_view.rotation) >= a_rules.minTurnCosine && Covers(a_view, a_cascade, h.box);
		}

		std::array<Drawn, kMaxCascades> history{};
		uint32_t count = 0;
		uint64_t frame = 0;
	};
}
