#include <iostream>
#include "hmmparser.h"

//////
// Testing code

class ChunkOfGarbage: public ChunkTBC<ChunkOfGarbage>
{
private:
	double continue_probability;

	class Factory
	{
	private:
		double continue_probability;
	public:
		Factory(double prob)
		: continue_probability(prob)
		{}

		ChunkOfGarbage operator() (char c) const
		{
			return ChunkOfGarbage(continue_probability);
		}
	};
public:
	ChunkOfGarbage(double continue_probability)
	: continue_probability(continue_probability)
	{}

	virtual const std::list<Transition<ChunkOfGarbage>> next() const
	{
		std::list<Transition<ChunkOfGarbage>> res;
		res.push_back(Transition<ChunkOfGarbage>(create(continue_probability), continue_probability));
		return res;
	}

	static ChunkState<ChunkOfGarbage>::Ptr create(double continue_probability)
	{
		return transform<ChunkOfGarbage>(SingleCharChunkState::create(), Factory(continue_probability));
	}
};

ChunkState<ChunkOfGarbage>::Ptr garbageTBC(double continue_probability)
{
	return continuation<ChunkOfGarbage>(ChunkOfGarbage::create(continue_probability));
}

int char_to_int(char c)
{
	return c;
}

int main(int argc, char* argv[])
{
	//std::shared_ptr<ChunkState<int>> start_state = transform<int>(SingleCharChunkState::create(), char_to_int);
	//auto start_state = garbage(0.99);
	//auto start_state = one_in_sequence<0>(Nil().then(number_impl::ScientificPartState::create(0.5)).then(literal("\n")));
	auto start_state = one_in_sequence<0>(Nil().then(a_floating_point_number(true, 0.5)).then(literal("\n")));
	//auto start_state = sequence(Nil().then(SingleCharChunkState::create()).then(SingleCharChunkState::create()).then(SingleCharChunkState::create()));
//	auto start_state = star(
//		sequence(Nil().
//			then(garbage(0.01)).
//			then(literal("Hello")).
//			then(whitespace()).
//			then(literal("world")).
//			then(whitespace()).
//			then(a_floating_point_number(true, 0.5)).
//			then(garbage(0.01))
//		),
//		0, -1
//	);
	//auto start_state = sequence(Nil().then(garbage(0.99)).then(an_integer()).then(garbage(0.99)));
	//auto start_state = a_positive_integer();
	auto chunk = parse(std::cin, start_state);
	std::cout << "Parse successful." << std::endl;
//	for( auto element : chunk )
//	{
//		std::cout << element.at<5>() << std::endl;
//	}
	std::cout << chunk << std::endl;
	return 0;
}
