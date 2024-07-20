#include "TransformationHelper.h"

#include <Eigen/Geometry>

namespace Transformation
{
	TransformationMeta::TransformationMeta(AxisAlignment right, AxisAlignment forward, AxisAlignment up,
		Ratio scale)
		: scale(scale), m_right(right), m_forward(forward), m_up(up)
	{
		if (right.axis == forward.axis || forward.axis == up.axis || right.axis == up.axis)
			throw std::exception("The same axis occurs twice!");
	}

	TransformationMeta::TransformationMeta(const TransformationMeta& other)
		: scale(other.scale), m_right(other.m_right), m_forward(other.m_forward), m_up(other.m_up)
	{
		if (other.right_handed)
			right_handed = std::make_unique<bool>(*other.right_handed);
	}

	bool TransformationMeta::isRightHanded() const
	{
		if (right_handed)
			return *right_handed;

		auto cpy = *this;

		//1. normalize to positive X axis as right axis
		if (m_right.axis == Axis::X)
		{
			//compare
		}
		else if (m_forward.axis == Axis::X)
		{
			cpy.rotateLeft();
		}
		else //up.axis == Axis::X
		{
			cpy.rotateRight();
		}
		if (cpy.m_right.direction == AxisDirection::NEGATIVE)
		{
			cpy.m_right.direction = AxisDirection::POSITIVE;
			cpy.m_forward.direction = invert(cpy.m_forward.direction);
		}
		const bool fw_up_eq = cpy.m_forward.direction == cpy.m_up.direction;

		//if (X, Y, Z) then the sign of Y and Z must be equal otherwise they are different
		//e.g. (X, Z, Y)
		right_handed = std::make_unique<bool>((cpy.m_forward.axis == Axis::Y) ? fw_up_eq : !fw_up_eq);
		return *right_handed;
	}

	/**
	 * convenience method
	 */
	bool TransformationMeta::isLeftHanded() const
	{
		return !isRightHanded();
	}

	void TransformationConverter::get_conv_matrix(Matrix3Interface<float>& out) const
	{
		for (const auto& [column, row, multiplier] : assignments)
		{
			//doesn't matter if we go over x or y for the result
			//only matters for cache performance
			for (int8_t y = 0; y < 3; ++y)
			{
				if (y == row)
					out(row, column) = multiplier * factor;
				else
					out(y, column) = 0.f;
			}
		}
	}
	/*
	Eigen::Matrix4f TransformationConverter::convert_matrix(const Matrix4Interface<float>& in) const
	{
		Eigen::Matrix4f out;
		Matrix4Eigen proxy{ out };
		convert(assignments, in, proxy, factor);

		return out;
	}
	*/
	void TransformationConverter::convert_matrix(const Matrix4Interface<float>& in, Matrix4Interface<float>& out) const
	{
		convert(assignments, in, out, factor);
	}

	void TransformationConverter::convert_point(const Vector3Interface<float>& in, Vector3Interface<float>& out) const
	{
		for (const auto& [column, row, multiplier] : assignments)
			out(row, in(column) * factor * multiplier);
	}
	/*
	Eigen::Quaternion<float> TransformationConverter::convert_quaternion(const Eigen::Quaternion<float>& in) const
	{
		Eigen::Quaternion<float> out;
		
		if (hand_changed)
			out.w() = -in.w();
		else
			out.w() = in.w();

		auto& coeffs = out.coeffs();
		const auto& in_c = in.coeffs();

		for (const auto& [column, row, multiplier] : assignments)
			coeffs[row] = in_c[column] * multiplier;
		
		return out;
	}

	Eigen::Vector3f TransformationConverter::convert_point(const Eigen::Vector3f& in) const
	{
		Eigen::Vector3f out;

		for (const auto& [column, row, multiplier] : assignments)
			out(row, 0) = in(column, 0) * factor * multiplier;

		return out;
	}

	pcl::PointXYZ TransformationConverter::convert_point_proto(const generated::vertex_3d& in_f) const
	{
		pcl::PointXYZ out;

		const auto in = std::to_array({ in_f.x(), in_f.y(), in_f.z() });

		for (const auto& [column, row, multiplier] : assignments)
			out.data[row] = in[column] * factor * multiplier;

		return out;
	}

	Eigen::Vector3f TransformationConverter::convert_point_proto_eigen(const generated::vertex_3d& in_f) const
	{
		Eigen::Vector3f out;

		const auto in = std::to_array({ in_f.x(), in_f.y(), in_f.z() });

		for (const auto& [column, row, multiplier] : assignments)
			out.data()[row] = in[column] * factor * multiplier;

		return out;
	}

	Eigen::Quaternionf TransformationConverter::convert_quaternion_proto(const generated::quaternion& in) const
	{
		Eigen::Quaternionf out;

		if (hand_changed)
			out.w() = -in.w();
		else
			out.w() = in.w();
		
		const auto in_c = std::to_array<float>({ in.x(), in.y(), in.z() });

		for (const auto& [column, row, multiplier] : assignments)
			out.coeffs()[row] = in_c[column] * multiplier;

		return out;
	}

	Eigen::Vector3f TransformationConverter::convert_size_proto(const generated::size_3d& in_f) const
	{
		Eigen::Vector3f out;

		const auto in = std::to_array({ in_f.x(), in_f.y(), in_f.z() });

		for (const auto& [column, row, multiplier] : assignments)
			out[row] = in[column] * factor;

		return out;
	}
	*/

	void TransformationConverter::convert_size(const Size3Interface<float>& in, Size3Interface<float>& out) const
	{
		for (const auto& [column, row, multiplier] : assignments)
			out(row, in(column) * factor);
	}

	float TransformationConverter::convert_scale(float scale) const
	{
		return factor * scale;
	}
	
	void TransformationConverter::convert_quaternion(const QuaternionInterface<float>& in,
		QuaternionInterface<float>& out) const
	{
		if (hand_changed)
			out.set_w(-in.get_w());
		else
			out.set_w(in.get_w());

		for (const auto& [column, row, multiplier] : assignments)
			out(row, in(column) * multiplier);
	}

	void TransformationConverter::convert(const SparseAssignments& ttt, const Matrix4Interface<float>& in, Matrix4Interface<float>& out, float scale)
	{
		for (size_t x = 0; x < 3; ++x)
			out(3, x) = 0.f;
		out(3, 3) = 1.f;

		for (size_t y = 0; y < 3; ++y)
		{
			//column == y
			const auto& [column, out_row, multiplier_y] = ttt[y];

			for (size_t x = 0; x < 3; ++x)
			{
				//exploiting symmetry //row == x
				const auto& [row, out_column, multiplier_x] = ttt[x];
				out(out_row, out_column) = in(y, x) * multiplier_y * multiplier_x;
			}
			out(out_row, 3) = in(y, 3) * multiplier_y * scale;
		}
	}

	void TransformationMeta::rotateLeft()
	{
		const auto tmp = m_right;
		m_right = m_forward;
		m_forward = m_up;
		m_up = tmp;
	}

	void TransformationMeta::rotateRight()
	{
		const auto tmp = m_up;
		m_up = m_forward;
		m_forward = m_right;
		m_right = tmp;
	}

	Assignment compute_assignment(AxisAlignment axis, AxisAlignment target_axis)
	{
		//column, row, multiplier
		return {
			static_cast<int8_t>(axis.axis),
			static_cast<int8_t>(target_axis.axis),
			static_cast<float>(axis.direction) * static_cast<float>(target_axis.direction)
		};
	}

	SparseAssignments compute_assignments(const TransformationMeta& origin, const TransformationMeta& target)
	{
		SparseAssignments ttt;
		auto tmp = compute_assignment(origin.right(), target.right());
		ttt[std::get<0>(tmp)] = std::move(tmp);

		tmp = compute_assignment(origin.forward(), target.forward());
		ttt[std::get<0>(tmp)] = std::move(tmp);

		tmp = compute_assignment(origin.up(), target.up());
		ttt[std::get<0>(tmp)] = std::move(tmp);

		return ttt;
	}

	AxisDirection invert(AxisDirection in)
	{
		if (in == AxisDirection::POSITIVE)
			return AxisDirection::NEGATIVE;
		return AxisDirection::POSITIVE;
	}

	Ratio::Ratio(std::intmax_t Num, std::intmax_t Denom)
		: Num(Num), Denom(Denom)
	{}

	float Ratio::factor() const
	{
		return static_cast<float>(Num) / static_cast<float>(Denom);
	}

	float Ratio::factor(const Ratio& other) const
	{
		return static_cast<float>(Num * other.Denom) / static_cast<float>(Denom * other.Num);
	}

	void QuaternionEigen::set_x(float x)
	{
		quaternion.x() = x;
	}

	float QuaternionEigen::get_x() const
	{
		return quaternion.x();
	}

	void QuaternionEigen::set_y(float y)
	{
		quaternion.y() = y;
	}

	float QuaternionEigen::get_y() const
	{
		return quaternion.y();
	}

	void QuaternionEigen::set_z(float z)
	{
		quaternion.z() = z;
	}

	float QuaternionEigen::get_z() const
	{
		return quaternion.z();
	}

	void QuaternionEigen::set_w(float w)
	{
		quaternion.w() = w;
	}

	float QuaternionEigen::get_w() const
	{
		return quaternion.w();
	}

	void QuaternionEigen::operator()(size_t idx, float value)
	{
		quaternion.coeffs().coeffRef(idx) = value;
	}

	float QuaternionEigen::operator()(size_t idx) const
	{
		return quaternion.coeffs().coeffRef(idx);
	}

	void QuaternionProto::set_x(float x)
	{
		quaternion.set_x(x);
	}

	float QuaternionProto::get_x() const
	{
		return quaternion.x();
	}

	void QuaternionProto::set_y(float y)
	{
		quaternion.set_y(y);
	}

	float QuaternionProto::get_y() const
	{
		return quaternion.y();
	}

	void QuaternionProto::set_z(float z)
	{
		quaternion.set_z(z);
	}

	float QuaternionProto::get_z() const
	{
		return quaternion.z();
	}

	void QuaternionProto::set_w(float w)
	{
		quaternion.set_w(w);
	}

	float QuaternionProto::get_w() const
	{
		return quaternion.w();
	}

	void QuaternionProto::operator()(size_t idx, float value)
	{
		(quaternion.*setter[idx])(value);
	}

	float QuaternionProto::operator()(size_t idx) const
	{
		return (quaternion.*getter[idx])();
	}

	std::array<float(generated::quaternion::*)() const, 4> QuaternionProto::getter = {
			&generated::quaternion::x,
			&generated::quaternion::y,
			&generated::quaternion::z,
			&generated::quaternion::w
	};

	std::array<void(generated::quaternion::*)(float), 4> QuaternionProto::setter = {
			&generated::quaternion::set_x,
			&generated::quaternion::set_y,
			&generated::quaternion::set_z,
			&generated::quaternion::set_w
	};

	TransformationConverter::TransformationConverter(const TransformationMeta& origin, const TransformationMeta& target)
		: factor(origin.scale.factor(target.scale)), assignments(compute_assignments(origin, target)), hand_changed(origin.isRightHanded() != target.isRightHanded())
	{}

	const AxisAlignment& TransformationMeta::right() const
	{
		return m_right;
	}

	const AxisAlignment& TransformationMeta::forward() const
	{
		return m_forward;
	}

	const AxisAlignment& TransformationMeta::up() const
	{
		return m_up;
	}

	float& Matrix4Eigen::operator()(size_t row, size_t column)
	{
		//maybe include range check
		return matrix(row, column);
	}
	const float& Matrix4Eigen::operator()(size_t row, size_t column) const
	{
		//maybe include range check
		return matrix(row, column);
	}

	void Vector3Eigen::set_x(float x)
	{
		vector.x() = x;
	}
	float Vector3Eigen::get_x() const
	{
		return vector.x();
	}
	void Vector3Eigen::set_y(float y)
	{
		vector.y() = y;
	}
	float Vector3Eigen::get_y() const
	{
		return vector.y();
	}
	void Vector3Eigen::set_z(float z)
	{
		vector.z() = z;
	}
	float Vector3Eigen::get_z() const
	{
		return vector.z();
	}

	void Vector3Eigen::operator()(size_t idx, float value)
	{
		vector(idx, 0) = value;
	}

	float Vector3Eigen::operator()(size_t idx) const
	{
		return vector(idx, 0);
	}

	void Vector3PCL::set_x(float x)
	{
		vector.x = x;
	}

	float Vector3PCL::get_x() const
	{
		return vector.x;
	}

	void Vector3PCL::set_y(float y)
	{
		vector.y = y;
	}

	float Vector3PCL::get_y() const
	{
		return vector.y;
	}

	void Vector3PCL::set_z(float z)
	{
		vector.z = z;
	}

	float Vector3PCL::get_z() const
	{
		return vector.z;
	}

	void Vector3PCL::operator()(size_t idx, float value)
	{
		vector.data[idx] = value;
	}

	float Vector3PCL::operator()(size_t idx) const
	{
		return vector.data[idx];
	}
	void Vector3Proto::set_x(float x)
	{
		vector.set_x(x);
	}

	float Vector3Proto::get_x() const
	{
		return vector.x();
	}

	void Vector3Proto::set_y(float y)
	{
		vector.set_y(y);
	}

	float Vector3Proto::get_y() const
	{
		return vector.y();
	}

	void Vector3Proto::set_z(float z)
	{
		vector.set_z(z);
	}

	float Vector3Proto::get_z() const
	{
		return vector.z();
	}

	void Vector3Proto::operator()(size_t idx, float value)
	{
		(vector.*setter[idx])(value);
	}

	float Vector3Proto::operator()(size_t idx) const
	{
		return (vector.*getter[idx])();
	}

	std::array<float(generated::vertex_3d::*)() const, 3> Vector3Proto::getter = {
			&generated::vertex_3d::x,
			&generated::vertex_3d::y,
			&generated::vertex_3d::z
	};

	std::array<void(generated::vertex_3d::*)(float), 3> Vector3Proto::setter = {
			&generated::vertex_3d::set_x,
			&generated::vertex_3d::set_y,
			&generated::vertex_3d::set_z
	};

	void Size3Eigen::set_x(float x)
	{
		vector.x() = x;
	}
	float Size3Eigen::get_x() const
	{
		return vector.x();
	}
	void Size3Eigen::set_y(float y)
	{
		vector.y() = y;
	}
	float Size3Eigen::get_y() const
	{
		return vector.y();
	}
	void Size3Eigen::set_z(float z)
	{
		vector.z() = z;
	}
	float Size3Eigen::get_z() const
	{
		return vector.z();
	}

	void Size3Eigen::operator()(size_t idx, float value)
	{
		vector(idx, 0) = value;
	}

	float Size3Eigen::operator()(size_t idx) const
	{
		return vector(idx, 0);
	}

	void Size3Proto::set_x(float x)
	{
		vector.set_x(x);
	}
	float Size3Proto::get_x() const
	{
		return vector.x();
	}
	void Size3Proto::set_y(float y)
	{
		vector.set_y(y);
	}
	float Size3Proto::get_y() const
	{
		return vector.y();
	}
	void Size3Proto::set_z(float z)
	{
		vector.set_z(z);
	}
	float Size3Proto::get_z() const
	{
		return vector.z();
	}

	void Size3Proto::operator()(size_t idx, float value)
	{
		(vector.*setter[idx])(value);
	}

	float Size3Proto::operator()(size_t idx) const
	{
		return (vector.*getter[idx])();
	}

	std::array<float(generated::size_3d::*)() const, 3> Size3Proto::getter = {
			&generated::size_3d::x,
			&generated::size_3d::y,
			&generated::size_3d::z
	};

	std::array<void(generated::size_3d::*)(float), 3> Size3Proto::setter = {
			&generated::size_3d::set_x,
			&generated::size_3d::set_y,
			&generated::size_3d::set_z
	};

	float& Matrix3Eigen::operator()(size_t row, size_t column)
	{
		return matrix(row, column);
	}

	const float& Matrix3Eigen::operator()(size_t row, size_t column) const
	{
		return matrix(row, column);
	}
}
