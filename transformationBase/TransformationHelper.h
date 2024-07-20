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

	template<typename ValueType>
	struct Matrix4Interface
	{
		virtual ~Matrix4Interface() = default;

		virtual ValueType& operator()(size_t row, size_t column) = 0;
		virtual const ValueType& operator()(size_t row, size_t column) const = 0;
	};

	template<typename ValueType>
	struct Matrix3Interface
	{
		virtual ~Matrix3Interface() = default;

		virtual ValueType& operator()(size_t row, size_t column) = 0;
		virtual const ValueType& operator()(size_t row, size_t column) const = 0;
	};

	template<typename ValueType>
	struct Vector3Interface
	{
		virtual ~Vector3Interface() = default;

		virtual void set_x(ValueType x) = 0;
		[[nodiscard]] virtual ValueType get_x() const = 0;

		virtual void set_y(ValueType y) = 0;
		[[nodiscard]] virtual ValueType get_y() const = 0;

		virtual void set_z(ValueType z) = 0;
		[[nodiscard]] virtual ValueType get_z() const = 0;

		//assumes [x,y,z]
		virtual void operator()(size_t idx, ValueType value) = 0;
		[[nodiscard]] virtual ValueType operator()(size_t idx) const = 0;
	};

	template<typename ValueType>
	struct Size3Interface
	{
		virtual ~Size3Interface() = default;

		virtual void set_x(ValueType x) = 0;
		[[nodiscard]] virtual ValueType get_x() const = 0;

		virtual void set_y(ValueType y) = 0;
		[[nodiscard]] virtual ValueType get_y() const = 0;

		virtual void set_z(ValueType z) = 0;
		[[nodiscard]] virtual ValueType get_z() const = 0;

		//assumes [x,y,z]
		virtual void operator()(size_t idx, ValueType value) = 0;
		[[nodiscard]] virtual ValueType operator()(size_t idx) const = 0;
	};

	class Matrix4Eigen : public Matrix4Interface<float>
	{
	public:

		Matrix4Eigen(Eigen::Matrix4f& matrix)
			: matrix(matrix)
		{}

		~Matrix4Eigen() override = default;

		float& operator()(size_t row, size_t column) override;
		const float& operator()(size_t row, size_t column) const override;

	private:

		Eigen::Matrix4f& matrix;
	};

	class Matrix3Eigen : public Matrix3Interface<float>
	{
	public:

		Matrix3Eigen(Eigen::Matrix3f& matrix)
			: matrix(matrix)
		{}

		~Matrix3Eigen() override = default;

		float& operator()(size_t row, size_t column) override;
		const float& operator()(size_t row, size_t column) const override;

	private:

		Eigen::Matrix3f& matrix;
	};

	class Vector3Eigen : public Vector3Interface<float>
	{
	public:

		Vector3Eigen(Eigen::Vector3f& vector)
			: vector(vector)
		{}

		~Vector3Eigen() override = default;

		void set_x(float x) override;
		[[nodiscard]] float get_x() const override;

		void set_y(float y) override;
		[[nodiscard]] float get_y() const override;

		void set_z(float z) override;
		[[nodiscard]] float get_z() const override;

		void operator()(size_t idx, float value) override;
		[[nodiscard]] float operator()(size_t idx) const override;

	private:

		Eigen::Vector3f& vector;
	};

	class Vector3PCL : public Vector3Interface<float>
	{
	public:

		Vector3PCL(pcl::PointXYZ& vector)
			: vector(vector)
		{}

		~Vector3PCL() override = default;

		void set_x(float x) override;
		[[nodiscard]] float get_x() const override;

		void set_y(float y) override;
		[[nodiscard]] float get_y() const override;

		void set_z(float z) override;
		[[nodiscard]] float get_z() const override;

		void operator()(size_t idx, float value) override;
		[[nodiscard]] float operator()(size_t idx) const override;

	private:

		pcl::PointXYZ& vector;
	};

	class Vector3Proto : public Vector3Interface<float>
	{
	public:

		Vector3Proto(generated::vertex_3d& vector)
			: vector(vector)
		{}

		~Vector3Proto() override = default;

		void set_x(float x) override;
		[[nodiscard]] float get_x() const override;

		void set_y(float y) override;
		[[nodiscard]] float get_y() const override;

		void set_z(float z) override;
		[[nodiscard]] float get_z() const override;

		void operator()(size_t idx, float value) override;
		[[nodiscard]] float operator()(size_t idx) const override;

	private:

		static std::array<float(generated::vertex_3d::*)() const, 3> getter;
		static std::array<void(generated::vertex_3d::*)(float), 3> setter;

		generated::vertex_3d& vector;
	};

	class Size3Eigen : public Size3Interface<float>
	{
	public:

		Size3Eigen(Eigen::Vector3f& vector)
			: vector(vector)
		{}

		~Size3Eigen() override = default;

		void set_x(float x) override;
		[[nodiscard]] float get_x() const override;

		void set_y(float y) override;
		[[nodiscard]] float get_y() const override;

		void set_z(float z) override;
		[[nodiscard]] float get_z() const override;

		void operator()(size_t idx, float value) override;
		[[nodiscard]] float operator()(size_t idx) const override;

	private:

		Eigen::Vector3f& vector;
	};

	class Size3Proto : public Size3Interface<float>
	{
	public:

		Size3Proto(generated::size_3d& vector)
			: vector(vector)
		{}

		~Size3Proto() override = default;

		void set_x(float x) override;
		[[nodiscard]] float get_x() const override;

		void set_y(float y) override;
		[[nodiscard]] float get_y() const override;

		void set_z(float z) override;
		[[nodiscard]] float get_z() const override;

		void operator()(size_t idx, float value) override;
		[[nodiscard]] float operator()(size_t idx) const override;

	private:

		static std::array<float(generated::size_3d::*)() const, 3> getter;
		static std::array<void(generated::size_3d::*)(float), 3> setter;

		generated::size_3d& vector;
	};

	template<typename ValueType>
	struct QuaternionInterface
	{
		virtual ~QuaternionInterface() = default;

		virtual void set_x(ValueType x) = 0;
		[[nodiscard]] virtual ValueType get_x() const = 0;

		virtual void set_y(ValueType y) = 0;
		[[nodiscard]] virtual ValueType get_y() const = 0;

		virtual void set_z(ValueType z) = 0;
		[[nodiscard]] virtual ValueType get_z() const = 0;

		virtual void set_w(ValueType z) = 0;
		[[nodiscard]] virtual ValueType get_w() const = 0;

		//assumes [x,y,z, w]
		virtual void operator()(size_t idx, ValueType value) = 0;
		[[nodiscard]] virtual ValueType operator()(size_t idx) const = 0;
	};

	class QuaternionEigen : public QuaternionInterface<float>
	{
	public:

		QuaternionEigen(Eigen::Quaternion<float>& quaternion)
			: quaternion(quaternion)
		{}

		~QuaternionEigen() override = default;

		void set_x(float x) override;
		[[nodiscard]] float get_x() const override;

		void set_y(float y) override;
		[[nodiscard]] float get_y() const override;

		void set_z(float z) override;
		[[nodiscard]] float get_z() const override;

		void set_w(float w) override;
		[[nodiscard]] float get_w() const override;

		void operator()(size_t idx, float value) override;
		[[nodiscard]] float operator()(size_t idx) const override;

	private:

		Eigen::Quaternion<float>& quaternion;
	};

	class QuaternionProto : public QuaternionInterface<float>
	{
	public:

		QuaternionProto(generated::quaternion& quaternion)
			: quaternion(quaternion)
		{}

		~QuaternionProto() override = default;

		void set_x(float x) override;
		[[nodiscard]] float get_x() const override;

		void set_y(float y) override;
		[[nodiscard]] float get_y() const override;

		void set_z(float z) override;
		[[nodiscard]] float get_z() const override;

		void set_w(float w) override;
		[[nodiscard]] float get_w() const override;

		void operator()(size_t idx, float value) override;
		[[nodiscard]] float operator()(size_t idx) const override;

	private:

		static std::array<float(generated::quaternion::*)() const, 4> getter;
		static std::array<void(generated::quaternion::*)(float), 4> setter;

		generated::quaternion& quaternion;
	};

	class TransformationConverter
	{
	public:

		TransformationConverter(const TransformationMeta& origin, const TransformationMeta& target);

		void get_conv_matrix(Matrix3Interface<float>& out) const;
		/*
		[[nodiscard]] Eigen::Matrix4f convert_matrix(const Matrix4Interface<float>& in) const;
		

		[[nodiscard]] Eigen::Quaternion<float> convert_quaternion(const Eigen::Quaternion<float>& in) const;
		[[nodiscard]] Eigen::Vector3f convert_point(const Eigen::Vector3f& in) const;

		[[nodiscard]] pcl::PointXYZ convert_point_proto(const generated::vertex_3d& in) const;
		[[nodiscard]] Eigen::Vector3f convert_point_proto_eigen(const generated::vertex_3d& in) const;
		[[nodiscard]] Eigen::Quaternionf convert_quaternion_proto(const generated::quaternion& in) const;
		[[nodiscard]] Eigen::Vector3f convert_size_proto(const generated::size_3d& in) const;
		*/

		[[nodiscard]] float convert_scale(float scale) const;

		void convert_quaternion(const QuaternionInterface<float>& in, QuaternionInterface<float>& out) const;
		void convert_matrix(const Matrix4Interface<float>& in, Matrix4Interface<float>& out) const;
		void convert_point(const Vector3Interface<float>& in, Vector3Interface<float>& out) const;
		void convert_size(const Size3Interface<float>& in, Size3Interface<float>& out) const;

	private:

		static void convert(const SparseAssignments& ttt, const Matrix4Interface<float>& in, Matrix4Interface<float>& out, float scale);

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