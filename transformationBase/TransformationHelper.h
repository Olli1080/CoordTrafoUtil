#pragma once
#include <cstdint>
#include <memory>
#include <ratio>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <pcl/impl/point_types.hpp>

#include <vertex.pb.h>

namespace Transformation
{
	enum class Axis : int8_t
	{
		X = 0,
		Y = 1,
		Z = 2
	};

	enum class AxisDirection : int8_t
	{
		POSITIVE = +1,
		NEGATIVE = -1
	};

	[[nodiscard]] AxisDirection invert(AxisDirection in);

	enum class TransformOperation
	{
		RIGHT_AND_FORWARD,
		RIGHT_AND_UP,
		FORWARD_AND_UP
	};

	struct AxisAlignment
	{
		Axis axis;
		AxisDirection direction;
	};

	struct Ratio
	{
		std::intmax_t Num;
		std::intmax_t Denom;

		Ratio(std::intmax_t Num, std::intmax_t Denom);

		template<std::intmax_t Num, std::intmax_t Denom>
		inline Ratio()
			: Num(Num), Denom(Denom)
		{}

		template<std::intmax_t Num, std::intmax_t Denom>
		inline Ratio(std::ratio<Num, Denom> ratio)
			: Num(Num), Denom(Denom)
		{}

		[[nodiscard]] float factor() const;
		[[nodiscard]] float factor(const Ratio& other) const;
	};

	//column, row, multiplier
	typedef std::tuple<int8_t, int8_t, float> Assignment;
	typedef std::array<Assignment, 3> SparseAssignments;

	class TransformationMeta;
	static Assignment compute_assignment(AxisAlignment axis, AxisAlignment target_axis);
	static SparseAssignments compute_assignments(const TransformationMeta& origin, const TransformationMeta& target);

	class TransformationConverter
	{
	public:

		TransformationConverter(const TransformationMeta& origin, const TransformationMeta& target);

		[[nodiscard]] Eigen::Matrix3f get_conv_matrix() const;

		[[nodiscard]] Eigen::Matrix4f convert_matrix(const Eigen::Matrix4f& in) const;
		[[nodiscard]] Eigen::Quaternion<float> convert_quaternion(const Eigen::Quaternion<float>& in) const;
		[[nodiscard]] Eigen::Vector3f convert_point(const Eigen::Vector3f& in) const;

		[[nodiscard]] pcl::PointXYZ convert_point_proto(const generated::vertex_3d& in) const;
		[[nodiscard]] Eigen::Vector3f convert_point_proto_eigen(const generated::vertex_3d& in) const;
		[[nodiscard]] Eigen::Quaternionf convert_quaternion_proto(const generated::quaternion& in) const;
		[[nodiscard]] Eigen::Vector3f convert_size_proto(const generated::size_3d& in) const;

		[[nodiscard]] float convert_scale(float scale) const;

	private:

		static Eigen::Matrix4f convert(const SparseAssignments& ttt, const Eigen::Matrix4f& in, float scale);

		float factor;
		SparseAssignments assignments;
		bool hand_changed;
	};

	class TransformationMeta
	{
	public:

		TransformationMeta(
			AxisAlignment right,
			AxisAlignment forward,
			AxisAlignment up,
			Ratio scale = { 1, 1 }
		);

		TransformationMeta(const TransformationMeta& other);

		[[nodiscard]] bool isRightHanded() const;
		[[nodiscard]] bool isLeftHanded() const;

		Ratio scale;

		const AxisAlignment& right() const;
		const AxisAlignment& forward() const;
		const AxisAlignment& up() const;

	private:

		void rotateLeft();
		void rotateRight();

		AxisAlignment m_right;
		AxisAlignment m_forward;
		AxisAlignment m_up;

		mutable std::unique_ptr<bool> right_handed;
	};

	inline static Transformation::TransformationMeta CoreMeta(
		{ Transformation::Axis::Y, Transformation::AxisDirection::POSITIVE },
		{ Transformation::Axis::X, Transformation::AxisDirection::NEGATIVE },
		{ Transformation::Axis::Z, Transformation::AxisDirection::POSITIVE }
	);
}