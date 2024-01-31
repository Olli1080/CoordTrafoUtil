
//#include <cstdint>
#include <iostream>
//#include <tuple>
//#include <numbers>

#include <Eigen/Dense>

#include "TransformationHelper.h"

/**
 * right handed combinations
 *			
 x,  y,  z		
 x, -z,  y		 
 x,  z, -y		 
 x, -y, -z		 

-x,  y, -z
-x, -z, -y
-x, -y,  z
-x,  z,  y 

 y,  x, -z
 y, -z, -x
 y, -x,  z
 y,  z,  x 

-y,  x,  z
-y, -z,  x
-y, -x, -z
-y,  z, -x

 z,  y, -x
 z,  x,  y
 z, -y,  x
 z, -x, -y

-z,  y,  x
-z,  x, -y
-z, -y, -x
-z, -x,  y

 */
//
//
//template<Axis ax, AxisDirection Direction>
//struct AxisAlignment
//{
//	static constexpr auto axis = ax;
//	static constexpr auto direction = Direction;
//};
//
//template <Axis right_ax, AxisDirection right_dir,
//	Axis forward_ax, AxisDirection forward_dir,
//	Axis up_ax, AxisDirection up_dir>
//struct TransformationMeta
//{
//	AxisAlignment<right_ax, right_dir> right;
//	AxisAlignment<forward_ax, forward_dir> forward;
//	AxisAlignment<up_ax, up_dir> up;
//
//	static TransformationMeta to_runtime()
//	{
//		return
//		{
//			{ right_ax, right_dir },
//			{ forward_ax, forward_dir },
//			{ up_ax, up_dir }
//		};
//	}
//};
//
//template<AxisDirection dir>
//constexpr AxisDirection invert()
//{
//	if constexpr (dir == AxisDirection::POSITIVE)
//		return AxisDirection::NEGATIVE;
//	else
//		return AxisDirection::POSITIVE;
//}
//
//template<Axis ax, AxisDirection dir>
//constexpr AxisAlignment<ax, invert<dir>()> invert(AxisAlignment<ax, dir> in)
//{
//	return {};
//}
//
//template<Axis ax_0, AxisDirection dir_0, Axis ax_1, AxisDirection dir_1, Axis ax_2, AxisDirection dir_2>
//constexpr TransformationMeta<ax_1, dir_1, ax_0, dir_0, ax_2, dir_2> swap_xy(TransformationMeta<ax_0, dir_0, ax_1, dir_1, ax_2, dir_2> in)
//{
//	return {};
//}
//
//template<Axis ax_0, AxisDirection dir_0, Axis ax_1, AxisDirection dir_1, Axis ax_2, AxisDirection dir_2>
//constexpr TransformationMeta<ax_2, dir_2, ax_1, dir_1, ax_0, dir_0> swap_xz(TransformationMeta<ax_0, dir_0, ax_1, dir_1, ax_2, dir_2> in)
//{
//	return {};
//}
//
//template<Axis ax_0, AxisDirection dir_0, Axis ax_1, AxisDirection dir_1, Axis ax_2, AxisDirection dir_2>
//constexpr TransformationMeta<ax_0, dir_0, ax_2, dir_2, ax_1, dir_1> swap_yz(TransformationMeta<ax_0, dir_0, ax_1, dir_1, ax_2, dir_2> in)
//{
//	return {};
//}
//
//template<TransformOperation op, Axis ax_0, AxisDirection dir_0, Axis ax_1, AxisDirection dir_1, Axis ax_2, AxisDirection dir_2>
//constexpr auto swap_axis(TransformationMeta<ax_0, dir_0, ax_1, dir_1, ax_2, dir_2> in)
//{
//	if constexpr (op == TransformOperation::RIGHT_AND_FORWARD)
//		return swap_xy(in);
//	else if constexpr (op == TransformOperation::RIGHT_AND_UP)
//		return swap_xz(in);
//	else if constexpr (op == TransformOperation::FORWARD_AND_UP)
//		return swap_yz(in);
//}
//
///**
// * \brief operand which is to be inverted
// * \param aa0 left operand
// * \param aa1 right operand
// */
//template<TransformOperation op, Axis ax_0, AxisDirection dir_0, Axis ax_1, AxisDirection dir_1, Axis ax_2, AxisDirection dir_2>
//constexpr auto swap_left(TransformationMeta<ax_0, dir_0, ax_1, dir_1, ax_2, dir_2> in)
//{
//	auto [right, forward, up] = swap_axis<op>(in);
//
//	if constexpr (op == TransformOperation::RIGHT_AND_FORWARD)
//		return TransformationMeta<right.axis, invert<right.direction>(), forward.axis, forward.direction, up.axis, up.direction>{};
//	else if constexpr (op == TransformOperation::RIGHT_AND_UP)
//		return TransformationMeta<right.axis, invert<right.direction>(), forward.axis, forward.direction, up.axis, up.direction>{};
//	else if constexpr (op == TransformOperation::FORWARD_AND_UP)
//		return TransformationMeta<right.axis, right.direction, forward.axis, invert<forward.direction>(), up.axis, up.direction>{};
//}
//
//template<TransformOperation op, Axis ax_0, AxisDirection dir_0, Axis ax_1, AxisDirection dir_1, Axis ax_2, AxisDirection dir_2>
//constexpr auto swap_right(TransformationMeta<ax_0, dir_0, ax_1, dir_1, ax_2, dir_2> in)
//{
//	auto [right, forward, up] = swap_axis<op>(in);
//
//	if constexpr (op == TransformOperation::RIGHT_AND_FORWARD)
//		return TransformationMeta<right.axis, right.direction, forward.axis, invert<forward.direction>(), up.axis, up.direction>{};
//	else if constexpr (op == TransformOperation::RIGHT_AND_UP)
//		return TransformationMeta<right.axis, right.direction, forward.axis, forward.direction, up.axis, invert<up.direction>()>{};
//	else if constexpr (op == TransformOperation::FORWARD_AND_UP)
//		return TransformationMeta<right.axis, right.direction, forward.axis, forward.direction, up.axis, invert<up.direction>()>{};
//}
//
//template<TransformOperation op, Axis ax_0, AxisDirection dir_0, Axis ax_1, AxisDirection dir_1, Axis ax_2, AxisDirection dir_2>
//constexpr auto invert(TransformationMeta<ax_0, dir_0, ax_1, dir_1, ax_2, dir_2> in)
//{
//	if constexpr (op == TransformOperation::RIGHT_AND_FORWARD)
//		return TransformationMeta<ax_0, invert<dir_0>(), ax_1, invert<dir_1>(), ax_2, dir_2>{};
//	else if constexpr (op == TransformOperation::RIGHT_AND_UP)
//		return TransformationMeta<ax_0, invert<dir_0>(), ax_1, dir_1, ax_2, invert<dir_2>()>{};
//	else if constexpr (op == TransformOperation::FORWARD_AND_UP)
//		return TransformationMeta<ax_0, dir_0, ax_1, invert<dir_1>(), ax_2, invert<dir_2>()>{};
//}
///*
//template<class trafoMeta>
//constexpr bool is_right_handed()
//{
//	auto xyz_0 = TransformationMeta<Axis::X, AxisDirection::POSITIVE, Axis::Y, AxisDirection::POSITIVE, Axis::Z, AxisDirection::POSITIVE>{}; //x,  y,  z
//	auto xyz_1 = swap_left<TransformOperation::FORWARD_AND_UP>(xyz_0);	// x, -z,  y
//	auto xyz_2 = swap_right<TransformOperation::FORWARD_AND_UP>(xyz_0);	// x,  z, -y	
//	auto xyz_3 = invert<TransformOperation::FORWARD_AND_UP>(xyz_0);		// x, -y, -z
//
//	auto xyz_4 = invert<TransformOperation::RIGHT_AND_UP>(xyz_0);	//-x,  y, -z
//	auto xyz_5 = invert<TransformOperation::RIGHT_AND_UP>(xyz_1);	//-x, -z, -y
//	auto xyz_6 = invert<TransformOperation::RIGHT_AND_UP>(xyz_2);	//-x,  z,  y 
//	auto xyz_7 = invert<TransformOperation::RIGHT_AND_UP>(xyz_3);	//-x, -y,  z
//
//	auto yxz_0 = swap_left<TransformOperation::RIGHT_AND_FORWARD>(xyz_3); // y,  x, -z
//	auto yxz_1 = swap_left<TransformOperation::FORWARD_AND_UP>(yxz_0);	// y,  z,  x 
//	auto yxz_2 = swap_right<TransformOperation::FORWARD_AND_UP>(yxz_0);	// y, -z, -x
//	auto yxz_3 = invert<TransformOperation::FORWARD_AND_UP>(yxz_0);		// y, -x,  z
//
//	auto yxz_4 = invert<TransformOperation::RIGHT_AND_UP>(yxz_0);	//-y,  x,  z
//	auto yxz_5 = invert<TransformOperation::RIGHT_AND_UP>(yxz_1);	//-y,  z, -x
//	auto yxz_6 = invert<TransformOperation::RIGHT_AND_UP>(yxz_2);	//-y, -z,  x
//	auto yxz_7 = invert<TransformOperation::RIGHT_AND_UP>(yxz_3);	//-y, -x, -z
//
//	auto zyx_0 = swap_right<TransformOperation::RIGHT_AND_UP>(xyz_0);	// z,  y, -x
//	auto zyx_1 = swap_left<TransformOperation::FORWARD_AND_UP>(zyx_0);	// z,  x,  y
//	auto zyx_2 = swap_right<TransformOperation::FORWARD_AND_UP>(zyx_0);	// z, -x, -y
//	auto zyx_3 = invert<TransformOperation::FORWARD_AND_UP>(zyx_0);		// z, -y,  x
//
//	auto zyx_4 = invert<TransformOperation::RIGHT_AND_UP>(zyx_0);	//-z,  y,  x
//	auto zyx_5 = invert<TransformOperation::RIGHT_AND_UP>(zyx_1);	//-z,  x, -y
//	auto zyx_6 = invert<TransformOperation::RIGHT_AND_UP>(zyx_2);	//-z, -y, -x
//	auto zyx_7 = invert<TransformOperation::RIGHT_AND_UP>(zyx_3);	//-z, -x,  y
//
//
//	return false;
//};*/
//
//
//
//
//template<typename T, Axis axis, AxisDirection dir>
//constexpr T component(const Eigen::Vector<T, 3>& v)
//{
//	float val;
//	if constexpr (axis == Axis::X)
//		val = v.x();
//	else if constexpr (axis == Axis::Y)
//		val = v.y();
//	else if constexpr (axis == Axis::Z)
//		val = v.z();
//	else
//		static_assert(true, "Invalid!");
//
//	return static_cast<T>(dir) * val;
//}
//
//template<typename T, Axis ax_0, AxisDirection dir_0, Axis ax_1, AxisDirection dir_1, Axis ax_2, AxisDirection dir_2>
//constexpr Eigen::Vector<T, 3> from_trafo(const Eigen::Vector<T, 3>& in, TransformationMeta<ax_0, dir_0, ax_1, dir_1, ax_2, dir_2> tm)
//{
//	Eigen::Vector3f out;
//	out.x() = component<T, ax_0, dir_0>(in);
//	out.y() = component<T, ax_1, dir_1>(in);
//	out.z() = component<T, ax_2, dir_2>(in);
//
//	return out;
//}
//
//void print(const Eigen::Vector3f& v)
//{
//	std::cout << "[ " << v.x() << " , " << v.y() << " , " << v.z() << " ]" << std::endl;
//}

using namespace Transformation;

int main()
{
/*	auto xyz_0 = TransformationMeta<Axis::X, AxisDirection::POSITIVE, Axis::Y, AxisDirection::POSITIVE, Axis::Z, AxisDirection::POSITIVE>{}; //x,  y,  z
	auto xyz_1 = swap_left<TransformOperation::FORWARD_AND_UP>(xyz_0);	// x, -z,  y
	auto xyz_2 = swap_right<TransformOperation::FORWARD_AND_UP>(xyz_0);	// x,  z, -y	
	auto xyz_3 = invert<TransformOperation::FORWARD_AND_UP>(xyz_0);		// x, -y, -z

	auto xyz_4 = invert<TransformOperation::RIGHT_AND_UP>(xyz_0);	//-x,  y, -z
	auto xyz_5 = invert<TransformOperation::RIGHT_AND_UP>(xyz_1);	//-x, -z, -y
	auto xyz_6 = invert<TransformOperation::RIGHT_AND_UP>(xyz_2);	//-x,  z,  y 
	auto xyz_7 = invert<TransformOperation::RIGHT_AND_UP>(xyz_3);	//-x, -y,  z

	auto yxz_0 = swap_left<TransformOperation::RIGHT_AND_FORWARD>(xyz_3); // y,  x, -z
	auto yxz_1 = swap_left<TransformOperation::FORWARD_AND_UP>(yxz_0);	// y,  z,  x 
	auto yxz_2 = swap_right<TransformOperation::FORWARD_AND_UP>(yxz_0);	// y, -z, -x
	auto yxz_3 = invert<TransformOperation::FORWARD_AND_UP>(yxz_0);		// y, -x,  z

	auto yxz_4 = invert<TransformOperation::RIGHT_AND_UP>(yxz_0);	//-y,  x,  z
	auto yxz_5 = invert<TransformOperation::RIGHT_AND_UP>(yxz_1);	//-y,  z, -x
	auto yxz_6 = invert<TransformOperation::RIGHT_AND_UP>(yxz_2);	//-y, -z,  x
	auto yxz_7 = invert<TransformOperation::RIGHT_AND_UP>(yxz_3);	//-y, -x, -z

	auto zyx_0 = swap_right<TransformOperation::RIGHT_AND_UP>(xyz_0);	// z,  y, -x
	auto zyx_1 = swap_left<TransformOperation::FORWARD_AND_UP>(zyx_0);	// z,  x,  y
	auto zyx_2 = swap_right<TransformOperation::FORWARD_AND_UP>(zyx_0);	// z, -x, -y
	auto zyx_3 = invert<TransformOperation::FORWARD_AND_UP>(zyx_0);		// z, -y,  x

	auto zyx_4 = invert<TransformOperation::RIGHT_AND_UP>(zyx_0);	//-z,  y,  x
	auto zyx_5 = invert<TransformOperation::RIGHT_AND_UP>(zyx_1);	//-z,  x, -y
	auto zyx_6 = invert<TransformOperation::RIGHT_AND_UP>(zyx_2);	//-z, -y, -x
	auto zyx_7 = invert<TransformOperation::RIGHT_AND_UP>(zyx_3);	//-z, -x,  y



	auto vec = Eigen::Vector3f(1, 2, 3);

	print(from_trafo(vec, xyz_0));
	print(from_trafo(vec, xyz_1));
	print(from_trafo(vec, xyz_2));
	print(from_trafo(vec, xyz_3));
	std::cout << std::endl;
	print(from_trafo(vec, xyz_4));
	print(from_trafo(vec, xyz_5));
	print(from_trafo(vec, xyz_6));
	print(from_trafo(vec, xyz_7));
	std::cout << std::endl;
	print(from_trafo(vec, yxz_0));
	print(from_trafo(vec, yxz_1));
	print(from_trafo(vec, yxz_2));
	print(from_trafo(vec, yxz_3));
	std::cout << std::endl;
	print(from_trafo(vec, yxz_4));
	print(from_trafo(vec, yxz_5));
	print(from_trafo(vec, yxz_6));
	print(from_trafo(vec, yxz_7));
	std::cout << std::endl;
	print(from_trafo(vec, zyx_0));
	print(from_trafo(vec, zyx_1));
	print(from_trafo(vec, zyx_2));
	print(from_trafo(vec, zyx_3));
	std::cout << std::endl;
	print(from_trafo(vec, zyx_4));
	print(from_trafo(vec, zyx_5));
	print(from_trafo(vec, zyx_6));
	print(from_trafo(vec, zyx_7));
	//std::cout << from_trafo(vec, yxz) << std::endl;

	std::cout << xyz_0.to_runtime().isRightHanded() << std::endl;
	std::cout << xyz_1.to_runtime().isRightHanded() << std::endl;
	std::cout << xyz_2.to_runtime().isRightHanded() << std::endl;
	std::cout << xyz_3.to_runtime().isRightHanded() << std::endl;

	std::cout << xyz_4.to_runtime().isRightHanded() << std::endl;
	std::cout << xyz_5.to_runtime().isRightHanded() << std::endl;
	std::cout << xyz_6.to_runtime().isRightHanded() << std::endl;
	std::cout << xyz_7.to_runtime().isRightHanded() << std::endl;

	std::cout << yxz_0.to_runtime().isRightHanded() << std::endl;
	std::cout << yxz_1.to_runtime().isRightHanded() << std::endl;
	std::cout << yxz_2.to_runtime().isRightHanded() << std::endl;
	std::cout << yxz_3.to_runtime().isRightHanded() << std::endl;

	std::cout << yxz_4.to_runtime().isRightHanded() << std::endl;
	std::cout << yxz_5.to_runtime().isRightHanded() << std::endl;
	std::cout << yxz_6.to_runtime().isRightHanded() << std::endl;
	std::cout << yxz_7.to_runtime().isRightHanded() << std::endl;

	std::cout << zyx_0.to_runtime().isRightHanded() << std::endl;
	std::cout << zyx_1.to_runtime().isRightHanded() << std::endl;
	std::cout << zyx_2.to_runtime().isRightHanded() << std::endl;
	std::cout << zyx_3.to_runtime().isRightHanded() << std::endl;

	std::cout << zyx_4.to_runtime().isRightHanded() << std::endl;
	std::cout << zyx_5.to_runtime().isRightHanded() << std::endl;
	std::cout << zyx_6.to_runtime().isRightHanded() << std::endl;
	std::cout << zyx_7.to_runtime().isRightHanded() << std::endl;
	std::cout << std::endl << std::endl;

	auto xzy_0 = TransformationMeta<Axis::X, AxisDirection::POSITIVE, Axis::Z, AxisDirection::POSITIVE, Axis::Y, AxisDirection::POSITIVE>{};
	auto xzy_1 = swap_left<TransformOperation::FORWARD_AND_UP>(xzy_0);	// x, -z,  y
	auto xzy_2 = swap_right<TransformOperation::FORWARD_AND_UP>(xzy_0);	// x,  z, -y	
	auto xzy_3 = invert<TransformOperation::FORWARD_AND_UP>(xzy_0);		// x, -y, -z

	auto xzy_4 = invert<TransformOperation::RIGHT_AND_UP>(xzy_0);	//-x,  y, -z
	auto xzy_5 = invert<TransformOperation::RIGHT_AND_UP>(xzy_1);	//-x, -z, -y
	auto xzy_6 = invert<TransformOperation::RIGHT_AND_UP>(xzy_2);	//-x,  z,  y 
	auto xzy_7 = invert<TransformOperation::RIGHT_AND_UP>(xzy_3);	//-x, -y,  z

	auto left_yxz_0 = swap_left<TransformOperation::RIGHT_AND_FORWARD>(xzy_3); // y,  x, -z
	auto left_yxz_1 = swap_left<TransformOperation::FORWARD_AND_UP>(left_yxz_0);	// y,  z,  x 
	auto left_yxz_2 = swap_right<TransformOperation::FORWARD_AND_UP>(left_yxz_0);	// y, -z, -x
	auto left_yxz_3 = invert<TransformOperation::FORWARD_AND_UP>(left_yxz_0);		// y, -x,  z

	auto left_yxz_4 = invert<TransformOperation::RIGHT_AND_UP>(left_yxz_0);	//-y,  x,  z
	auto left_yxz_5 = invert<TransformOperation::RIGHT_AND_UP>(left_yxz_1);	//-y,  z, -x
	auto left_yxz_6 = invert<TransformOperation::RIGHT_AND_UP>(left_yxz_2);	//-y, -z,  x
	auto left_yxz_7 = invert<TransformOperation::RIGHT_AND_UP>(left_yxz_3);	//-y, -x, -z

	auto left_zyx_0 = swap_right<TransformOperation::RIGHT_AND_UP>(xzy_0);	// z,  y, -x
	auto left_zyx_1 = swap_left<TransformOperation::FORWARD_AND_UP>(left_zyx_0);	// z,  x,  y
	auto left_zyx_2 = swap_right<TransformOperation::FORWARD_AND_UP>(left_zyx_0);	// z, -x, -y
	auto left_zyx_3 = invert<TransformOperation::FORWARD_AND_UP>(left_zyx_0);		// z, -y,  x

	auto left_zyx_4 = invert<TransformOperation::RIGHT_AND_UP>(left_zyx_0);	//-z,  y,  x
	auto left_zyx_5 = invert<TransformOperation::RIGHT_AND_UP>(left_zyx_1);	//-z,  x, -y
	auto left_zyx_6 = invert<TransformOperation::RIGHT_AND_UP>(left_zyx_2);	//-z, -y, -x
	auto left_zyx_7 = invert<TransformOperation::RIGHT_AND_UP>(left_zyx_3);	//-z, -x,  y


	std::cout << xzy_0.to_runtime().isRightHanded() << std::endl;
	std::cout << xzy_1.to_runtime().isRightHanded() << std::endl;
	std::cout << xzy_2.to_runtime().isRightHanded() << std::endl;
	std::cout << xzy_3.to_runtime().isRightHanded() << std::endl;

	std::cout << xzy_4.to_runtime().isRightHanded() << std::endl;
	std::cout << xzy_5.to_runtime().isRightHanded() << std::endl;
	std::cout << xzy_6.to_runtime().isRightHanded() << std::endl;
	std::cout << xzy_7.to_runtime().isRightHanded() << std::endl;

	std::cout << left_yxz_0.to_runtime().isRightHanded() << std::endl;
	std::cout << left_yxz_1.to_runtime().isRightHanded() << std::endl;
	std::cout << left_yxz_2.to_runtime().isRightHanded() << std::endl;
	std::cout << left_yxz_3.to_runtime().isRightHanded() << std::endl;

	std::cout << left_yxz_4.to_runtime().isRightHanded() << std::endl;
	std::cout << left_yxz_5.to_runtime().isRightHanded() << std::endl;
	std::cout << left_yxz_6.to_runtime().isRightHanded() << std::endl;
	std::cout << left_yxz_7.to_runtime().isRightHanded() << std::endl;

	std::cout << left_zyx_0.to_runtime().isRightHanded() << std::endl;
	std::cout << left_zyx_1.to_runtime().isRightHanded() << std::endl;
	std::cout << left_zyx_2.to_runtime().isRightHanded() << std::endl;
	std::cout << left_zyx_3.to_runtime().isRightHanded() << std::endl;

	std::cout << left_zyx_4.to_runtime().isRightHanded() << std::endl;
	std::cout << left_zyx_5.to_runtime().isRightHanded() << std::endl;
	std::cout << left_zyx_6.to_runtime().isRightHanded() << std::endl;
	std::cout << left_zyx_7.to_runtime().isRightHanded() << std::endl;
	*/

	TransformationMeta source = 
	{
		{Axis::X, AxisDirection::POSITIVE},
		{Axis::Y, AxisDirection::NEGATIVE},
		{Axis::Z, AxisDirection::NEGATIVE},
	};

	TransformationMeta target =
	{
		{ Axis::X, AxisDirection::POSITIVE },
		{ Axis::Z, AxisDirection::POSITIVE },
		{ Axis::Y, AxisDirection::POSITIVE },
		std::centi{}
	};

	Eigen::Matrix4f tempMat;
	tempMat << 0, 1, 2, 3,
		10, 11, 12, 13,
		20, 21, 22, 23,
		0, 0, 0, 1;

	TransformationConverter source_to_target(source, target);

	std::cout << source_to_target.get_conv_matrix() << std::endl << std::endl;
	std::cout << source_to_target.convert_matrix(tempMat) << std::endl << std::endl;

	TransformationMeta source_2 = 
	{
		{ Axis::Y, AxisDirection::NEGATIVE },
		{ Axis::Z, AxisDirection::NEGATIVE },
		{ Axis::X, AxisDirection::POSITIVE }
	};

	TransformationConverter source_2_to_target(source_2, target);

	std::cout << source_2_to_target.get_conv_matrix() << std::endl << std::endl;
	std::cout << source_2_to_target.convert_matrix(tempMat) << std::endl << std::endl;
	
	std::cout << source_2.isLeftHanded() << std::endl << std::endl;
	

	TransformationMeta source_3 =
	{
		{ Axis::Y, AxisDirection::NEGATIVE },
		{ Axis::X, AxisDirection::POSITIVE },
		{ Axis::Z, AxisDirection::POSITIVE }
	};

	TransformationMeta unity =
	{
		{ Axis::X, AxisDirection::POSITIVE },
		{ Axis::Z, AxisDirection::POSITIVE },
		{ Axis::Y, AxisDirection::POSITIVE }
	};

	TransformationConverter source_3_to_unity(source_3, unity);

	Eigen::Quaternion<float> quat; quat = Eigen::AngleAxisf(-2.237f, Eigen::Vector3f::UnitX())
		* Eigen::AngleAxisf(-2.217f, Eigen::Vector3f::UnitY())
		* Eigen::AngleAxisf(-0.030f, Eigen::Vector3f::UnitZ());
	std::cout << quat << std::endl;
	std::cout << source_3_to_unity.convert_quaternion(quat).normalized() << std::endl << std::endl;

	std::cout << source_3_to_unity.get_conv_matrix() << std::endl;
	std::cout << source_3_to_unity.convert_point(Eigen::Vector3f{ 1, 2, 3 }) << std::endl;

	return 0;
}