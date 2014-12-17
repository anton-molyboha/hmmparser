#include <iostream>
#include <memory>
#include <list>
#include <stdexcept>

template<typename Chunk>
struct Transition;

template<typename Chunk>
class ChunkState;

template<typename Chunk>
class ChunkState
{
public:
	typedef std::shared_ptr<ChunkState<Chunk>> Ptr;
	typedef char CharT;
	typedef Chunk ChunkT;
	typedef Transition<ChunkT> TransitionT;

public:
	/**
	 * The probability to generate character c.
	 * The total of all probabilities over the characters c, plus getProbablityFinished() should be 1.0
	 */
	virtual double getProbability(CharT c) const = 0;

	/**
	 * The probability to finish generation
	 */
	virtual double getProbabilityFinished() const = 0;

	/**
	 * Return the list of transitions conditional on having generated character c.
	 * A transition is a state that may follow this state, and the probability with
	 * which that state will follow.
	 */
	virtual const std::list<TransitionT> getTransitions(CharT c) const = 0;

	/**
	 * Assume the chunk is finished and generate the object representing the chunk.
	 */
	virtual ChunkT finish() const = 0;

	/**
	 * Virtual destructor needed for our use of inheritance and polymorphism
	 */
	virtual ~ChunkState() {}
};

template<typename Chunk>
struct Transition
{
	typedef Chunk ChunkT;

	typename ChunkState<ChunkT>::Ptr state;
	double prob;

	Transition(typename ChunkState<ChunkT>::Ptr state, double prob)
	:state(state), prob(prob)
	{}

	Transition<Chunk> operator* (double prob_multiplier)
	{
		return Transition<Chunk>(state, prob * prob_multiplier);
	}

	Transition<Chunk> operator/ (double prob_multiplier)
	{
		return Transition<Chunk>(state, prob / prob_multiplier);
	}

	template<typename ChunkStateT>
	Transition<typename ChunkStateT::ChunkT> with_state(std::shared_ptr<ChunkStateT> other_state)
	{
		return Transition<typename ChunkStateT::ChunkT>(other_state, prob);
	}
};

template<typename Chunk>
Chunk parse(std::istream& strm, std::shared_ptr<ChunkState<Chunk>> state)
{
	typedef Transition<Chunk> TransitionT;
	typedef ChunkState<Chunk> ChunkStateT;

	std::list<TransitionT> states;
	states.push_back(TransitionT(state, 1.0));
	while(strm)
	{
		std::list<TransitionT> next;

		char chr = strm.get();
		if( strm )
		{
			for( auto it = states.begin(); it != states.end(); ++it )
			{
				double prob = it->prob * it->state->getProbability(chr);
				if( prob > 0 )
				{
					std::list<TransitionT> trans = it->state->getTransitions(chr);
					for( auto it2 = trans.begin(); it2 != trans.end(); ++it2 )
					{
						next.push_back(TransitionT(it2->state, it2->prob * prob));
					}
				}
			}
		}
		else
		{
			for( auto it = states.begin(); it != states.end(); ++it )
			{
				double prob = it->prob * it->state->getProbabilityFinished();
				if( prob > 0 )
				{
					next.push_back(TransitionT(it->state, it->prob * prob));
				}
			}
		}

		double totalProb = 0;
		for( auto it = next.begin(); it != next.end(); ++it )
		{
			totalProb += it->prob;
		}
		for( auto it = next.begin(); it != next.end(); ++it )
		{
			it->prob /= totalProb;
		}
		
		states = next;
	}

	if( states.empty() )
	{
		throw std::runtime_error("Syntax error (could not parse)");
	}
	else
	{
		double maxProb = states.front().prob;
		typename ChunkStateT::Ptr res = states.front().state;
		for( auto it = states.begin(); it != states.end(); ++it )
		{
			if( it->prob > maxProb )
			{
				maxProb = it->prob;
				res = it->state;
			}
		}

		return res->finish();
	}
}

//////
// Basic chunks

template<typename Chunk>
class EmptyChunkState: public ChunkState<Chunk>, public std::enable_shared_from_this<EmptyChunkState<Chunk>>
{
public:
	typedef std::shared_ptr<EmptyChunkState<Chunk>> Ptr;
	typedef ChunkState<Chunk> StateT;
	typedef typename StateT::CharT CharT;
	typedef typename StateT::ChunkT ChunkT;
	typedef typename StateT::TransitionT TransitionT;

private:
	ChunkT chunk;

	EmptyChunkState(const ChunkT& chunk)
	: chunk(chunk)
	{}
	
public:
	static Ptr create(const ChunkT& chunk)
	{
		return Ptr(new EmptyChunkState<Chunk>(chunk));
	}

	virtual double getProbability(CharT c) const
	{
		return 0;
	}

	virtual double getProbabilityFinished() const
	{
		return 1;
	}

	virtual const std::list<TransitionT> getTransitions(CharT c) const
	{
		return std::list<TransitionT>();
	}

	virtual ChunkT finish() const
	{
		return chunk;
	}
};

class SingleCharChunkState: public ChunkState<char>, public std::enable_shared_from_this<SingleCharChunkState>
{
public:
	typedef std::shared_ptr<SingleCharChunkState> Ptr;

private:
	SingleCharChunkState()
	{}

public:
	static Ptr create()
	{
		return Ptr(new SingleCharChunkState());
	}

	virtual double getProbability(char c) const
	{
		return 1.0 / 256;
	}

	virtual double getProbabilityFinished() const
	{
		return 0;
	}

	virtual const std::list<Transition<char>> getTransitions(char c) const
	{
		std::list<Transition<char>> res;
		res.push_back(Transition<char>(EmptyChunkState<char>::create(c), 1));
		return res;
	}

	virtual char finish() const
	{
		throw std::logic_error("Call of ChunkState::finish() when getProbabilityFinished() == 0");
	}
};

class APositiveIntegerFirstChunkState;

class APositiveIntegerChunkState: public ChunkState<int>
{
public:
	typedef std::shared_ptr<APositiveIntegerChunkState> Ptr;
	typedef ChunkState<int>::CharT CharT;
	typedef ChunkState<int>::ChunkT ChunkT;
	typedef Transition<ChunkT> TransitionT;

private:
	double prob_finished;
	int prefix;

	APositiveIntegerChunkState(double prob_finished, int prefix)
	: prob_finished(prob_finished), prefix(prefix)
	{}

	static Ptr create(double prob_finished, int prefix)
	{
		return Ptr(new APositiveIntegerChunkState(prob_finished, prefix));
	}

	static Ptr create(double prob_finished)
	{
		return create(prob_finished, 0);
	}

	friend APositiveIntegerFirstChunkState;
public:
	virtual double getProbability(CharT c) const
	{
		if( (c >= '0') && (c <= '9') )
		{
			return (1 - prob_finished) / 10;
		}
		else
		{
			return 0;
		}
	}

	virtual double getProbabilityFinished() const
	{
		return prob_finished;
	}

	virtual const std::list<TransitionT> getTransitions(CharT c) const
	{
		if( (c >= '0') && (c <= '9') )
		{
			std::list<TransitionT> res;
			res.push_back(TransitionT(create(prob_finished, prefix * 10 + (c - '0')), 1));
			return res;
		}
		else
		{
			//throw std::logic_error(std::string("A call to APositiveInteger::getTransitions(") + c + ")");
			return std::list<TransitionT>();
		}
	}

	virtual int finish() const
	{
		return prefix;
	}
};

class APositiveIntegerFirstChunkState: public ChunkState<int>
{
private:
	double probability_finished;

	APositiveIntegerFirstChunkState(double prob_finished)
	: probability_finished(prob_finished)
	{}

public:
	typedef std::shared_ptr<APositiveIntegerFirstChunkState> Ptr;

	static Ptr create(double prob_finished)
	{
		return Ptr(new APositiveIntegerFirstChunkState(prob_finished));
	}

	static Ptr create()
	{
		return create(0.25);
	}

	virtual double getProbability(CharT c) const
	{
		if( (c >= '0') && (c <= '9') )
		{
			return 0.1;
		}
		else
		{
			return 0.0;
		}
	}

	virtual double getProbabilityFinished() const
	{
		return 0;
	}

	virtual const std::list<TransitionT> getTransitions(CharT c) const
	{
		std::list<TransitionT> res;
		if( (c >= '0') && (c <= '9') )
		{
			res.push_back(TransitionT(APositiveIntegerChunkState::create(probability_finished, c - '0'), 1));
		}
		return res;
	}

	virtual int finish() const
	{
		throw std::logic_error("APositiveIntegerFirstChunkState has zero probabilityFinished but finish() has been called");
	}
};

ChunkState<int>::Ptr a_positive_integer()
{
	return APositiveIntegerFirstChunkState::create();
}

//////
// Compositions of chunks

template<typename Chunk1, typename Chunk2, typename Transformation>
class TransformedChunkState: public ChunkState<Chunk2>, public std::enable_shared_from_this<TransformedChunkState<Chunk1, Chunk2, Transformation>>
{
public:
	typedef std::shared_ptr<TransformedChunkState<Chunk1, Chunk2, Transformation>> Ptr;

private:
	Transformation transformation;
	typename ChunkState<Chunk1>::Ptr base;

	TransformedChunkState(typename ChunkState<Chunk1>::Ptr base, Transformation transformation)
	: base(base), transformation(transformation)
	{}

public:
	static Ptr create(typename ChunkState<Chunk1>::Ptr base, Transformation transformation)
	{
		return Ptr(new TransformedChunkState<Chunk1, Chunk2, Transformation>(base, transformation));
	}

	virtual double getProbability(char c) const
	{
		return base->getProbability(c);
	}

	virtual double getProbabilityFinished() const
	{
		return base->getProbabilityFinished();
	}

	virtual const std::list<Transition<Chunk2>> getTransitions(char c) const
	{
		std::list<Transition<Chunk1>> base_transitions = base->getTransitions(c);
		std::list<Transition<Chunk2>> res;
		for( Transition<Chunk1> transition : base_transitions )
		{
			res.push_back(Transition<Chunk2>(create(transition.state, transformation), transition.prob));
		}
		return res;
	}

	virtual Chunk2 finish() const
	{
		return transformation(base->finish());
	}
};

template<typename Chunk2, typename ChunkStateT1, typename Transformation>
typename ChunkState<Chunk2>::Ptr transform(std::shared_ptr<ChunkStateT1> state, Transformation transformation)
{
	return TransformedChunkState<typename ChunkStateT1::ChunkT, Chunk2, Transformation>::create(state, transformation);
}

/**
 * A chunk type which knows what kind of chunk to read next
 */
template<typename NextChunk>
class ChunkTBC    // TBC stands for "to be continued"
{
public:
	typedef NextChunk NextChunkT;
	typedef Transition<NextChunkT> TransitionT;

	virtual const std::list<Transition<NextChunk>> next() const = 0;

	virtual double probabilityFinished() const
	{
		double res = 1;
		for( auto transition : next() )
		{
			res -= transition.prob;
		}
		return res;
	}
};

template<typename ToType, typename FromType>
class TypeCheckHelper
{
public:
	static ToType check(FromType arg)
	{
		throw std::runtime_error("Type mismatch");
	}
};

template<typename Type>
class TypeCheckHelper<Type, Type>
{
public:
	static Type check(Type arg)
	{
		return arg;
	}
};

template<typename ToType, typename FromType>
ToType type_check(FromType arg)
{
	return TypeCheckHelper<ToType, FromType>::check(arg);
}

/**
 * Converts a ChunkState whose Chunk type derives from ChunkTBC into
 * a ChunkState which will read not only this chunk, but also any continuation.
 */
template<typename LastChunk, typename CurChunk>
class ContinuedChunkState: public ChunkState<LastChunk>
{
public:
	typedef std::shared_ptr<ContinuedChunkState<LastChunk, CurChunk>> Ptr;
	typedef typename ChunkState<LastChunk>::CharT CharT;
	typedef typename ChunkState<LastChunk>::ChunkT ChunkT;
	typedef typename ChunkState<LastChunk>::TransitionT TransitionT;

private:
	typename ChunkState<CurChunk>::Ptr base;

	static constexpr double negligible = 1e-5;

	ContinuedChunkState(typename ChunkState<CurChunk>::Ptr base)
	: base(base)
	{}

	template<typename Chunk>
	double getProbabilityLoop(std::list<Transition<Chunk>> transitions, CharT c) const
	{
		typedef typename Chunk::NextChunkT NextChunk;
		std::list<Transition<NextChunk>> next;

		double res = 0;
		for( Transition<Chunk> transition : transitions )
		{
			res += transition.prob * transition.state->getProbability(c);
			double fprob = transition.prob * transition.state->getProbabilityFinished();
			if( fprob > negligible )
			{
				for( Transition<NextChunk> trans : transition.state->finish().next() )
				{
					next.push_back(Transition<NextChunk>(trans.state, trans.prob * fprob));
				}
			}
		}

		if( next.size() == 0 )
		{
			return res;
		}
		else
		{
			return res + getProbabilityLoop(next, c);
		}
	}

	template<typename Chunk>
	double probabilityFinishedLoop(std::list<Transition<Chunk>> transitions) const
	{
		typedef typename Chunk::NextChunkT NextChunk;
		std::list<Transition<NextChunk>> next;

		double res = 0;
		for( Transition<Chunk> transition : transitions )
		{
			double fprob = transition.prob * transition.state->getProbabilityFinished();
			if( fprob > negligible )
			{
				res += fprob * transition.state->finish().probabilityFinished();
				for( Transition<NextChunk> trans : transition.state->finish().next() )
				{
					next.push_back(trans * fprob);
				}
			}
		}

		if( next.size() == 0 )
		{
			return res;
		}
		else
		{
			return res + probabilityFinishedLoop(next);
		}
	}

	template<typename Chunk>
	const std::list<TransitionT> transitionsLoop(std::list<Transition<Chunk>> transitions, CharT c) const
	{
		std::list<TransitionT> res;
		std::list<Transition<typename Chunk::NextChunkT>> next;

		for( Transition<Chunk> transition : transitions )
		{
			double char_prob = transition.state->getProbability(c);
			if( char_prob > 0 )
			{
				for( Transition<Chunk> transition1 : transition.state->getTransitions(c) )
				{
					res.push_back(TransitionT(ContinuedChunkState<LastChunk, Chunk>::create(transition1.state), transition.prob * char_prob * transition1.prob));
				}
			}

			double fprob = transition.prob * transition.state->getProbabilityFinished();
			if( fprob > negligible )
			{
				for( Transition<typename Chunk::NextChunkT> trans : transition.state->finish().next() )
				{
					next.push_back(trans * fprob);
				}
			}
		}

		if( next.size() > 0 )
		{
			for( TransitionT trans : transitionsLoop(next, c) )
			{
				res.push_back(trans);
			}
		}

		return res;
	}

//	template<typename Chunk>
//	class FinishHelper
//	{
//	public:
//		static LastChunk finish(Chunk chunk)
//		{
//			throw runtime_error("Type mismatch in ContinuedChunkState.finish()");
//		}
//	};
//
//	template<>
//	class FinishHelper<LastChunk>
//	{
//	public:
//		static LastChunk finish(LastChunk chunk)
//		{
//			return chunk;
//		}
//	};

public:
	static Ptr create(typename ChunkState<CurChunk>::Ptr base)
	{
		return Ptr(new ContinuedChunkState(base));
	}

	virtual double getProbability(CharT c) const
	{
		double res = base->getProbability(c);
		double prob = base->getProbabilityFinished();
		if( prob > negligible )
		{
			std::list<Transition<typename CurChunk::NextChunkT>> next = base->finish().next();
			std::list<Transition<typename CurChunk::NextChunkT>> scaled;
			for( Transition<typename CurChunk::NextChunkT> transition : next )
			{
				scaled.push_back(Transition<typename CurChunk::NextChunkT>(transition.state, transition.prob * prob));
			}
			res += getProbabilityLoop(scaled, c);
		}
		return res;
	};

	virtual double getProbabilityFinished() const
	{
		double prob = base->getProbabilityFinished();
		if( prob > negligible )
		{
			double res = prob * base->finish().probabilityFinished();
			std::list<Transition<typename CurChunk::NextChunkT>> next = base->finish().next();
			std::list<Transition<typename CurChunk::NextChunkT>> scaled;
			for( Transition<typename CurChunk::NextChunkT> transition : next )
			{
				scaled.push_back(transition * prob);
			}
			res += probabilityFinishedLoop(scaled);
			return res;
		}
		else
		{
			return 0;
		}
	};

	virtual const std::list<TransitionT> getTransitions(CharT c) const
	{
		std::list<typename ChunkState<CurChunk>::TransitionT> transitions = base->getTransitions(c);
		std::list<TransitionT> res;
		for( typename ChunkState<CurChunk>::TransitionT transition : base->getTransitions(c) )
		{
			res.push_back(transition.with_state(ContinuedChunkState<LastChunk, CurChunk>::create(transition.state)));
		}

		double fprob = base->getProbabilityFinished();
		if( fprob > negligible )
		{
			std::list<Transition<typename CurChunk::NextChunkT>> next = base->finish().next();
			std::list<Transition<typename CurChunk::NextChunkT>> scaled;
			for( Transition<typename CurChunk::NextChunkT> transition : next )
			{
				scaled.push_back(transition * fprob);
			}
			for( TransitionT transition : transitionsLoop(scaled, c) )
			{
				res.push_back(transition);
			}
		}

		double total_prob = 0;
		for( auto transition : res )
		{
			total_prob += transition.prob;
		}

		std::list<TransitionT> scaled;
		for( auto transition : res )
		{
			scaled.push_back(transition / total_prob);
		}
		return scaled;
	}

	virtual ChunkT finish() const
	{
		// TODO: if type does not match, recurse into possible continuations
		return type_check<LastChunk>(base->finish());
	}
};

template<typename LastChunk, typename FirstChunkState>
typename ChunkState<LastChunk>::Ptr continuation(std::shared_ptr<FirstChunkState> start_state)
{
	typedef typename FirstChunkState::ChunkT FirstChunk;
	return ContinuedChunkState<LastChunk, FirstChunk>::create(start_state);
}

namespace tbc_helpers
{
	template<typename Chunk>
	class AsTBC: public ChunkTBC<AsTBC<Chunk>>
	{
	private:
		Chunk chunk;
	public:
		typedef Chunk WrappedChunkT;
		typedef AsTBC<Chunk> NextChunkT;

		AsTBC(Chunk chunk)
		: chunk(chunk)
		{}

		virtual const std::list<Transition<NextChunkT>> next() const
		{
			return std::list<Transition<NextChunkT>>();
		}

		Chunk get() const
		{
			return chunk;
		}
	};

	template<typename Chunk>
	AsTBC<Chunk> wrapper(Chunk chunk)
	{
		return AsTBC<Chunk>(chunk);
	}

	template<typename State>
	typename ChunkState<AsTBC<typename State::ChunkT>>::Ptr wrapTBC(std::shared_ptr<State> state)
	{
		return transform<AsTBC<typename State::ChunkT>>(state, wrapper<typename State::ChunkT>);
	}

	template<typename Chunk>
	Chunk unwrapper(AsTBC<Chunk> chunk)
	{
		return chunk.get();
	}

	template<typename State>
	typename ChunkState<typename State::ChunkT::WrappedChunkT>::Ptr unwrapTBC(std::shared_ptr<State> state)
	{
		typedef typename State::ChunkT::WrappedChunkT WrappedChunkT;
		return transform<WrappedChunkT>(state, unwrapper<WrappedChunkT>);
	}
}

using tbc_helpers::AsTBC;
using tbc_helpers::wrapTBC;
using tbc_helpers::unwrapTBC;

//////
// Basic chunks based on ChunkTBC

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

ChunkState<ChunkOfGarbage>::Ptr garbage(double continue_probability)
{
	return continuation<ChunkOfGarbage>(ChunkOfGarbage::create(continue_probability));
}

class SignChunkTBC: public ChunkTBC<AsTBC<int>>
{
private:
	bool negative;

public:
	typedef AsTBC<int> NextChunkT;

	SignChunkTBC(bool negative)
	: negative(negative)
	{}

	virtual const std::list<Transition<NextChunkT>> next() const
	{
		std::list<Transition<NextChunkT>> res;
		res.push_back(Transition<NextChunkT>(wrapTBC(transform<int>(a_positive_integer(), *this)), 1));
		return res;
	}

	int operator()(int value) const
	{
		if( negative )
		{
			return -value;
		}
		else
		{
			return value;
		}
	}
};

class SignChunkState: public ChunkState<SignChunkTBC>
{
private:
	SignChunkState() {}

public:
	typedef std::shared_ptr<SignChunkState> Ptr;

	static Ptr create()
	{
		return Ptr(new SignChunkState());
	}

	virtual double getProbability(CharT c) const
	{
		if( c == '-' )
		{
			return 0.5;
		}
		else
		{
			return 0;
		}
	}

	virtual double getProbabilityFinished() const
	{
		return 0.5;
	}

	virtual const std::list<TransitionT> getTransitions(CharT c) const
	{
		std::list<TransitionT> res;
		if( c == '-' )
		{
			res.push_back(TransitionT(EmptyChunkState<SignChunkTBC>::create(SignChunkTBC(true)), 1));
		}
		return res;
	}

	virtual SignChunkTBC finish() const
	{
		return SignChunkTBC(false);
	}
};

ChunkState<int>::Ptr an_integer()
{
	return unwrapTBC(continuation<AsTBC<int>>(SignChunkState::create()));
}

class FloatingPartChunkState: public ChunkState<double>
{
public:
	typedef std::shared_ptr<FloatingPartChunkState> Ptr;

private:
	double value;
	double order;
	double prob_finish;

	FloatingPartChunkState(double value, double order, double prob_finish)
	: value(value), order(order), prob_finish(prob_finish)
	{}

	static Ptr create(double value, double order, double prob_finish)
	{
		return Ptr(new FloatingPartChunkState(value, order, prob_finish));
	}

public:
	static Ptr create(double value)
	{
		return create(value, 0.1, 0.2);
	}

	virtual double getProbability(CharT c) const
	{
		if( (c >= '0') && (c <= '9') )
		{
			return (1 - prob_finish) / 10;
		}
		else
		{
			return 0;
		}
	}

	virtual double getProbabilityFinished() const
	{
		return prob_finish;
	}

	virtual const std::list<TransitionT> getTransitions(CharT c) const
	{
		std::list<TransitionT> res;
		if( (c >= '0') && (c <= '9') )
		{
			res.push_back(TransitionT(create(value + order * (c - '0'), order / 10, prob_finish), 1));
		}
		return res;
	}

	virtual ChunkT finish() const
	{
		return value;
	}
};

class FloatingPointChunkState: public ChunkState<double>
{
private:
	double integer_part;
	double probability_int;

	FloatingPointChunkState(double integer_part, double probability_int)
	: integer_part(integer_part), probability_int(probability_int)
	{}

public:
	typedef std::shared_ptr<FloatingPointChunkState> Ptr;

	static Ptr create(double integer_part, bool dot_is_optional)
	{
		if( dot_is_optional )
		{
			return Ptr(new FloatingPointChunkState(integer_part, 0.5));
		}
		else
		{
			return Ptr(new FloatingPointChunkState(integer_part, 0));
		}
	}

	virtual double getProbability(CharT c) const
	{
		if( c == '.' )
		{
			return 1 - probability_int;
		}
		else
		{
			return 0;
		}
	}

	virtual double getProbabilityFinished() const
	{
		return probability_int;
	}

	virtual const std::list<TransitionT> getTransitions(CharT c) const
	{
		std::list<TransitionT> res;
		if( c == '.' )
		{
			res.push_back(TransitionT(FloatingPartChunkState::create(integer_part), 1));
		}
		return res;
	}

	virtual ChunkT finish() const
	{
		return integer_part;
	}
};

class FirstChunkOfFloat: public ChunkTBC<AsTBC<double>>
{
private:
	int integer_part;
	bool dot_is_optional;

public:
	FirstChunkOfFloat(int integer_part, bool dot_is_optional)
	: integer_part(integer_part), dot_is_optional(dot_is_optional)
	{}

	class Builder
	{
	private:
		bool dot_is_optional;
	public:
		Builder(bool dot_is_optional)
		: dot_is_optional(dot_is_optional)
		{}

		FirstChunkOfFloat operator()(int integer_part) const
		{
			return FirstChunkOfFloat(integer_part, dot_is_optional);
		}
	};

	virtual const std::list<TransitionT> next() const
	{
		std::list<TransitionT> res;
		res.push_back(TransitionT(wrapTBC(FloatingPointChunkState::create(integer_part, dot_is_optional)), 1));
		return res;
	}
};

ChunkState<double>::Ptr a_floating_point_number(bool dot_is_optional)
{
	return unwrapTBC(continuation<AsTBC<double>>(transform<FirstChunkOfFloat>(an_integer(), FirstChunkOfFloat::Builder(dot_is_optional))));
}

//////
// List template

namespace list_template
{
	template<typename Head, typename Tail>
	class ListT;

	class Nil
	{
	public:
		template<typename T>
		class With
		{
		public:
			typedef ListT<T, Nil> Type;
		};

		template<typename Type>
		ListT<Type, Nil> then(Type value) const;
	};

	namespace impl
	{
		template<typename List, int index>
		class ListT_Getter;

		template<typename List, typename Type>
		class ListT_Builder;
	}

	template<typename Head, typename Tail>
	class ListT
	{
	public:
		typedef Head HeadT;
		typedef Tail TailT;
		typedef ListT<HeadT, TailT> SelfT;

		HeadT head;
		TailT tail;

		ListT(Head head, Tail tail)
		: head(head), tail(tail)
		{}

		template<int index>
		typename impl::ListT_Getter<SelfT, index>::TypeAt at() const;

		template<typename T>
		class With
		{
		public:
			typedef typename impl::ListT_Builder<SelfT, T>::ResType Type;
		};

		template<typename Type>
		typename With<Type>::Type then(Type value) const;
	};

	template<typename Head, typename Tail>
	ListT<Head, Tail> make_listT(Head head, Tail tail)
	{
		return ListT<Head, Tail>(head, tail);
	}

	template<typename Type>
	ListT<Type, Nil> listT(Type value)
	{
		return make_listT(value, Nil());
	}

	namespace impl
	{
		template<typename List, int index>
		class ListT_Getter
		{
		public:
			typedef typename ListT_Getter<typename List::TailT, index-1>::TypeAt TypeAt;
			static TypeAt get(const List& self)
			{
				return ListT_Getter<typename List::TailT, index-1>::get(self.tail);
			}
		};

		template<typename List>
		class ListT_Getter<List, 0>
		{
		public:
			typedef typename List::HeadT TypeAt;
			static TypeAt get(const List& self)
			{
				return self.head;
			}
		};

		template<typename List, typename Type>
		class ListT_Builder
		{
		public:
			typedef ListT<typename List::HeadT, typename ListT_Builder<typename List::TailT, Type>::ResType> ResType;
			static ResType then(const List& list, Type value)
			{
				return make_listT(list.head, ListT_Builder<typename List::TailT, Type>::then(list.tail, value));
			}
		};

		template<typename Type>
		class ListT_Builder<Nil, Type>
		{
		public:
			typedef ListT<Type, Nil> ResType;
			static ResType then(const Nil& list, Type value)
			{
				return make_listT(value, Nil());
			}
		};
	}

	template<typename Head, typename Tail>
	template<int index>
	typename impl::ListT_Getter<ListT<Head, Tail>, index>::TypeAt ListT<Head, Tail>::at() const
	{
		return impl::ListT_Getter<ListT<Head, Tail>, index>::get(*this);
	}

	template<typename Type>
	ListT<Type, Nil> Nil::then(Type value) const
	{
		return listT(value);
	}

	template<typename Head, typename Tail>
	template<typename Type>
	typename ListT<Head, Tail>::template With<Type>::Type ListT<Head, Tail>::then(Type value) const
	{
		return impl::ListT_Builder<ListT<Head, Tail>, Type>::then(*this, value);
	}
}

using list_template::ListT;
using list_template::Nil;
using list_template::listT;
using list_template::make_listT;

namespace sequence_impl
{
	template<typename StatePtr>
	class StateToChunkConverter
	{};

	template<typename StateT>
	class StateToChunkConverter<std::shared_ptr<StateT>>
	{
	public:
		typedef typename StateT::ChunkT ChunkT;
	};

	template<typename StateList>
	class StatesToChunksConverter
	{
	public:
		typedef ListT<typename StateToChunkConverter<typename StateList::HeadT>::ChunkT, typename StatesToChunksConverter<typename StateList::TailT>::Chunks> Chunks;
	};

	template<>
	class StatesToChunksConverter<Nil>
	{
	public:
		typedef Nil Chunks;
	};

	template<typename ParsedList, typename List>
	class ListChunk;

	template<typename ParsedList, typename List>
	class TypesHelper
	{
	public:
		typedef typename StateToChunkConverter<typename List::HeadT>::ChunkT NextRawChunk;
		typedef typename ParsedList::template With<NextRawChunk>::Type NextParsedList;
		typedef typename List::TailT NextList;
		typedef ListChunk<NextParsedList, NextList> NextChunk;
	};

	template<typename ParsedList, typename List>
	class ListChunk: public ChunkTBC<typename TypesHelper<ParsedList, List>::NextChunk>
	{
	private:
		ParsedList parsed_list;
		List list;
	public:
		typedef typename TypesHelper<ParsedList, List>::NextChunk NextChunk;

		ListChunk(ParsedList parsed_list, List list)
		: parsed_list(parsed_list), list(list)
		{}

		virtual const std::list<Transition<NextChunk>> next() const
		{
			std::list<Transition<NextChunk>> res;
			res.push_back(Transition<NextChunk>(transform<NextChunk>(list.head, *this), 1));
			return res;
		}

		NextChunk operator() (typename StateToChunkConverter<typename List::HeadT>::ChunkT chunk) const
		{
			return NextChunk(parsed_list.then(chunk), list.tail);
		}
	};

	template<typename ParsedList>
	class ListChunk<ParsedList, Nil>: public ChunkTBC<ListChunk<ParsedList, Nil>>
	{
	private:
		ParsedList parsed_list;
	public:
		typedef ListChunk<ParsedList, Nil> NextChunk;

		ListChunk(ParsedList parsed_list, Nil list)
		: parsed_list(parsed_list)
		{}

		virtual const std::list<Transition<NextChunk>> next() const
		{
			std::list<Transition<NextChunk>> res;
			return res;
		}

		ParsedList get_parsed() const
		{
			return parsed_list;
		}
	};

	template<typename ParsedList>
	ParsedList extractor(ListChunk<ParsedList, Nil> chunk)
	{
		return chunk.get_parsed();
	}

	template<typename StateList>
	typename ChunkState<typename StatesToChunksConverter<StateList>::Chunks>::Ptr sequence(StateList states)
	{
		typedef typename StatesToChunksConverter<StateList>::Chunks ParsedList;
		typedef ListChunk<ParsedList, Nil> LastChunk;
		typedef ListChunk<Nil, StateList> FirstChunk;
		
		typename ChunkState<LastChunk>::Ptr res = continuation<LastChunk>(transform<typename FirstChunk::NextChunkT>(states.head, FirstChunk(Nil(), states)));
		return transform<ParsedList>(res, extractor<ParsedList>);
	}
}

using sequence_impl::sequence;

//////
// Testing code

int char_to_int(char c)
{
	return c;
}

int main(int argc, char* argv[])
{
	//std::shared_ptr<ChunkState<int>> start_state = transform<int>(SingleCharChunkState::create(), char_to_int);
	//auto start_state = sequence(Nil().then(SingleCharChunkState::create()).then(SingleCharChunkState::create()).then(SingleCharChunkState::create()));
	auto start_state = sequence(Nil().then(garbage(0.99)).then(a_floating_point_number(false)).then(garbage(0.99)));
	//auto start_state = a_positive_integer();
	auto chunk = parse(std::cin, start_state);
	std::cout << "Parse successful." << std::endl;
	std::cout << chunk.at<1>() << std::endl;
	//std::cout << chunk << std::endl;
	return 0;
}
