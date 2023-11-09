// Copyright © 2021 Giorgio Audrito. All Rights Reserved.

/**
 * @file exercises.cpp
 * @brief Quick-start aggregate computing exercises.
 */

// [INTRODUCTION]
//! Importing the FCPP library.
#include "lib/fcpp.hpp"
#include <list>
#include "lib/deployment/hardware_identifier.hpp"
#include <cmath>

/**
 * @brief Namespace containing all the objects in the FCPP library.
 */
namespace fcpp {

//! @brief Dummy ordering between positions (allows positions to be used as secondary keys in ordered tuples).
template <size_t n>
bool operator<(vec<n> const&, vec<n> const&) {
    return false;
}

//! @brief Namespace containing the libraries of coordination routines.
namespace coordination {
//! @brief Tags used in the node storage.
namespace tags {
    //! @brief Color of the current node.
    struct node_color {};
    //! @brief Size of the current node.
    struct node_size {};
    //! @brief Shape of the current node.
    struct node_shape {};
    //! @brief number of neighbour of the current node.
    struct node_nbr {};
    // ... add more as needed, here and in the tuple_store<...> option below
    struct max_current_nbr {};
    struct max_node_nbr {};
	struct min_node_nbr {};
	struct min_node_uid {};
	struct node_uid {};
}

//! @brief The maximum communication range between nodes.
constexpr size_t communication_range = 100;

// [AGGREGATE PROGRAM]

/**
 * EXERCISES:
 *
 * Expand the MAIN function below to compute the following:
 *
 * 1)    The number of neighbour devices.
 *
 * 2)    The maximum number of neighbour devices ever witnessed by the current device.
 *
 * 3)    The maximum number of neighbour devices ever witnessed by any device in the network.
 *
 * 4)    Move towards the neighbour with the lowest number of neighbours.
 *
 * Every exercise above is designed to help solving the following one.
 *
 * In order to check whether what you computed is correct, you may display the computed
 * quantities as node qualities through tags `node_color`, `node_size` and `node_shape`.
 * You can also save your computed quantities in additional specific node attributes:
 * towards this end, you should both add a tag in namespace tags above, then list it
 * (together with the corresponding data type) in the `tuple_store` option below.
 *
 *
 * BONUS EXERCISES:
 *
 * 5)    Move away from the neighbour with the highest number of neighbours.
 *
 * 6)    Move as if the device was attracted by the neighbour with the lowest number of neighbours,
 *       and repulsed by the neighbour with the highest number of neighbours.
 *
 * 7)    Move as if the device was repulsed by every neighbour, and by the four walls of the
 *       rectangular box between points [0,0] and [500,500].
 *
 * HINTS:
 *
 * -    In the first few exercises, start by reasoning on when/where to use `nbr` (collecting from
 *      neighbours) and `old` (collecting from the past).
 *
 * -    In order to move a device, you need to set a velocity vector through something like
 *      `node.velocity() = make_vec(0,0)`.
 *
 * -    Coordinates are available through `node.position()`. Coordinates can be composed as physical
 *      vectors: `[1,3] + [2,-1] == [3,2]`, `[2,4] * 0.5 == [1,2]`.
 *
 * -    In the last two exercises, you can model attraction/repulsion using the classical inverse square law.
 *      More precisely, if `v` is the vector between two objects, the resulting force is `v / |v|^3` where
 *      `|v| = sqrt(v_x^2 + v_y^2)`. In FCPP, `norm(v)` is available for computing `|v|`.
 */

 FUN int maxNumberNeighbour(ARGS, int max_current){
    CODE

    return nbr(CALL, max_current, [&](field<int> max_field){
        return max_hood(CALL, max_field, max_current);
    });
}


fcpp::vec<2> getClosestPoint(float x, float y){
	if(x > 250){
		if(y > 250){
			if(x > y){
				return {500, y};
			}else{
				return {x, 500};
			}
		}else{
			if(x > 500 - y){
				return {500, y};
			}else{
				return {x, 0};
			}
		}
	}else{
		if(y > 250){
			if(500 - x > y){
				return {0, y};
			}else{
				return {x, 500};
			}
		}else{
			if(x < y){
				return {0, y};
			}else{
				return {x, 0};
			}
		}
	}
}



FUN field<fcpp::vec<2>> test(ARGS, fcpp::vec<2> sum_test){
    CODE
	
//	std::cout << "test inside function test test --" << node.uid << "--\n";
	field<fcpp::vec<2>> nbr_positions = nbr(CALL, node.position());
	
	
	field<fcpp::vec<2>> f = map_hood([&](fcpp::vec<2> p){
		fcpp::vec<2> force;
		fcpp::vec<2> v = p - node.position();
		
		if(node.uid == 17){
			std::cout << "test inside11: " << " - "<< p << " - "<< v << "\n";
		}
		
		if(norm(v) != 0){
			force = v / pow(norm(v), 3);
		}else{
			force = {0, 0};
		}
		
		p = force * -1;
		
		if(node.uid == 17){
			std::cout << "test inside: " << force << " - "<< p << " - "<< v << "\n";
		}
		sum_test += p;
		
		return p;
	}, nbr_positions);
	
	
	
	return f;
}

//! @brief Main function.
//using position_type = vec<fcpp::component::tags::dimension>;


MAIN() {
    // import tag names in the local scope.
    using namespace tags;
	
	int uid = node.uid;
	
    //ES 1
    field<int> nField = nbr(CALL, 1);
    int num = sum_hood(CALL, nField);

    //ES 2
    int max_current = old(CALL, 0, [&](int max){
        if(num > max){
            return num;
        }
        return max;
    });
   
    //ES 3
    int max_any = maxNumberNeighbour(CALL, max_current);

        
 
    //ES 4
	int min_nbr = 0;
	
	tuple<field<int>, field<int>> t = make_tuple(nbr(CALL, num), node.nbr_uid());
	tuple<int, int> min_t = min_hood(CALL, t);
	
	int min_uid = get<1>(min_t);
	min_nbr = get<0>(min_t);
	
	const node_t& min_node = node.net.node_at(min_uid);
	
	float angle_min = atan2(min_node.position()[1] - node.position()[1], min_node.position()[0] - node.position()[0]);
	
	if(min_uid != node.uid){
		//node.velocity() = {10 * cos(angle_min), 10 * sin(angle_min)};
	}else{
		//node.velocity() = {0,0};
	}
	
	
	//ES 5
	int max_nbr = 0;
	
	tuple<field<int>, field<int>> t2 = make_tuple(nbr(CALL, num), node.nbr_uid());
	tuple<int, int> max_t = max_hood(CALL, t2);
	
	int max_uid = get<1>(max_t);
	const node_t& max_node = node.net.node_at(get<1>(max_t));
	
	float angle_max = atan2(max_node.position()[1] - node.position()[1], max_node.position()[0] - node.position()[0]);
	
	if(max_uid != node.uid){
		//node.velocity() = {-10 * cos(angle_max), -10 * sin(angle_max)};
	}else{
		//node.velocity() = {0,0};
	}
	
	//ES 6
	
	fcpp::vec<2> attraction;
	
	if(min_uid != node.uid){
		fcpp::vec<2> v = min_node.position() - node.position();
		fcpp::vec<2> force = v / pow(norm(v), 2);

		attraction = force * 2;
		
		//node.velocity() += attraction;
	}
	
	if(max_uid != node.uid){
		fcpp::vec<2> v = max_node.position() - node.position();
		fcpp::vec<2> force = v / pow(norm(v), 2);

		fcpp::vec<2> repulsion = force * -2;
		
		//node.velocity() += repulsion;
	}

	
	//ES 7
	
	int x = node.position()[0];
	int y = node.position()[1];
	fcpp::vec<2> border_position = getClosestPoint(x, y);
	
	fcpp::vec<2> v = border_position - node.position();
	fcpp::vec<2> force = v / pow(norm(v), 3);

	fcpp::vec<2> border_force = force * -2;
	
	//node.velocity() += border_force;
	
	
	fcpp::vec<2> att = {0, 0};
	field<fcpp::vec<2>> nbr_positions = nbr(CALL, node.position());
	fcpp::vec<2> sum_test = {0,0};
	if(uid == 17){
		std::cout << "pos: " << sum_hood(CALL, nbr_positions) << "\n";
	}
	
	
	/*nbr_positions = map_hood([&](fcpp::vec<2> p){
		fcpp::vec<2> force;
		fcpp::vec<2> v = p - node.position();
		
		if(uid == 17){
			std::cout << "test inside11: " << " - "<< p << " - "<< v << "\n";
		}
		
		if(norm(v) != 0){
			force = v / pow(norm(v), 3);
		}else{
			force = {0, 0};
		}
		
		p = force * -1;
		if(uid == 17){
			std::cout << "test inside: " << force << " - "<< p << " - "<< v << "\n";
		}
		sum_test += p;
		
		return p;
	}, nbr_positions);*/
	
	if(uid == 17){
		//std::cout << "pos2: " << node.velocity() << " - " << sum_test << " - " << node.velocity() + sum_test << "\n";
	}
	
	att = sum_hood(CALL, test(CALL, sum_test));
	node.velocity() = node.velocity() + att;
	
	
	/*int counter = 0;
	fold_hood(CALL, [&](fcpp::vec<2> p1, fcpp::vec<2> p2){
		std::cout << "HERE\n";
		fcpp::vec<2> force, v;
		if(counter == 0){
			v = p2 - node.position();
			
			if(norm(v) != 0){
				force = v / pow(norm(v), 2);
			}else{
				force = {0, 0};
			}

			att = force * -2;
		}
		
		
		v = p1 - node.position();
		if(norm(v) != 0){
			force = v / pow(norm(v), 2);
		}else{
			force = {0, 0};
		}
		
		att += (force * -2);
		counter++;
		
		return att;
	}, nbr_positions);
	
	std::cout << "attasdfijo: " << att << "\n";*/
	
	//node.velocity() += att;
	
	// usage of node storage
    node.storage(node_size{}) = 10;
    node.storage(node_color{}) = color(GREEN);
    node.storage(node_shape{}) = shape::sphere;
    node.storage(node_nbr{}) = num;
    node.storage(max_current_nbr{}) = max_current;
    node.storage(max_node_nbr{}) = max_any;
	node.storage(min_node_nbr{}) = min_nbr;
	node.storage(min_node_uid{}) = min_uid;
	node.storage(node_uid{}) = uid;
}

//! @brief Export types used by the main function (update it when expanding the program).
FUN_EXPORT main_t = export_list<double, int, fcpp::vec<2>>;

} // namespace coordination

// [SYSTEM SETUP]

//! @brief Namespace for component options.
namespace option {

//! @brief Import tags to be used for component options.
using namespace component::tags;
//! @brief Import tags used by aggregate functions.
using namespace coordination::tags;

//! @brief Number of people in the area.
constexpr int node_num = 100;
//! @brief Dimensionality of the space.
constexpr size_t dim = 2;

//! @brief Description of the round schedule.
using round_s = sequence::periodic<
    distribution::interval_n<times_t, 0, 1>,    // uniform time in the [0,1] interval for start
    distribution::weibull_n<times_t, 10, 1, 10> // weibull-distributed time for interval (10/10=1 mean, 1/10=0.1 deviation)
>;
//! @brief The sequence of network snapshots (one every simulated second).
using log_s = sequence::periodic_n<1, 0, 1>;
//! @brief The sequence of node generation events (node_num devices all generated at time 0).
using spawn_s = sequence::multiple_n<node_num, 0>;
//! @brief The distribution of initial node positions (random in a 500x500 square).
using rectangle_d = distribution::rect_n<1, 0, 0, 500, 500>;
//! @brief The contents of the node storage as tags and associated types.
using store_t = tuple_store<
    node_color,                 color,
    node_size,                  double,
    node_shape,                 shape,
    node_nbr,                   int,
    max_current_nbr,            int,
    max_node_nbr,               int,
	min_node_nbr,				int,
	min_node_uid,				int,
	node_uid,					int
>;
//! @brief The tags and corresponding aggregators to be logged (change as needed).
using aggregator_t = aggregators<
    node_size,                  aggregator::mean<double>
>;

//! @brief The general simulation options.
DECLARE_OPTIONS(list,
    parallel<true>,      // multithreading enabled on node rounds
    synchronised<false>, // optimise for asynchronous networks
    program<coordination::main>,   // program to be run (refers to MAIN above)
    exports<coordination::main_t>, // export type list (types used in messages)
    retain<metric::retain<2,1>>,   // messages are kept for 2 seconds before expiring
    round_schedule<round_s>, // the sequence generator for round events on nodes
    log_schedule<log_s>,     // the sequence generator for log events on the network
    spawn_schedule<spawn_s>, // the sequence generator of node creation events on the network
    store_t,       // the contents of the node storage
    aggregator_t,  // the tags and corresponding aggregators to be logged
    init<
        x,      rectangle_d // initialise position randomly in a rectangle for new nodes
    >,
    dimension<dim>, // dimensionality of the space
    connector<connect::fixed<100, 1, dim>>, // connection allowed within a fixed comm range
    shape_tag<node_shape>, // the shape of a node is read from this tag in the store
    size_tag<node_size>,   // the size  of a node is read from this tag in the store
    color_tag<node_color>  // the color of a node is read from this tag in the store
);

} // namespace option

} // namespace fcpp


//! @brief The main function.
int main() {
    using namespace fcpp;

    //! @brief The network object type (interactive simulator with given options).
    using net_t = component::interactive_simulator<option::list>::net;
    //! @brief The initialisation values (simulation name).
    auto init_v = common::make_tagged_tuple<option::name>("Exercises");
    //! @brief Construct the network object.
    net_t network{init_v};
    //! @brief Run the simulation until exit.
    network.run();
    return 0;
}
