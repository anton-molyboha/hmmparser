#ifndef HMMPARSER_H
#define HMMPARSER_H 1

#include <memory>
#include <list>
#include <stdexcept>
#include <cmath>
#include "list2.h"

/**
 * typedef-only class template which selects which implementation of list and smart pointer to use throughout the code.
 */
template<typename T>
class Containers
{
public:
    typedef list2<T> list;
    typedef std::shared_ptr<T> ChunkPtr;    // This could have been some kind of intrusive_ptr, for example.
};

template<typename Chunk>
struct Transition;

template<typename Chunk>
class ChunkState;

/**
 * The hidden state of the imaginary Markov process generating the text we are looking at.
 */
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
    virtual const typename Containers<TransitionT>::list getTransitions(CharT c) const = 0;

    /**
     * Assume the chunk is finished and generate the object representing the chunk.
     */
    virtual ChunkT finish() const = 0;

    /**
     * Virtual destructor needed for our use of inheritance and polymorphism
     */
    virtual ~ChunkState() {}
};

/**
 * A structure that couples a ChunkState with the probability of being in that state.
 */
template<typename Chunk>
struct Transition
{
    typedef Chunk ChunkT;

    typename ChunkState<ChunkT>::Ptr state;
    double prob;

    Transition(typename ChunkState<ChunkT>::Ptr state, double prob);

    /**
     * Multiply the probability of this Transition by a coefficient
     */
    Transition<Chunk> operator * (double prob_multiplier) const;

    /**
     * Divide the probability of this Transition by a coefficient
     */
    Transition<Chunk> operator / (double prob_multiplier) const;

    /**
     * Multiply the probability of this Transition by a coefficient
     */
    Transition<Chunk>& operator *= (double prob_multiplier);

    /**
     * Divide the probability of this Transition by a coefficient
     */
    Transition<Chunk>& operator /= (double prob_multiplier);

    /**
     * Create a Transition with the same probability but a different ChunkState
     *
     * This is usually used when the state gets transformed in some way.
     */
    template<typename ChunkStateT>
    Transition<typename ChunkStateT::ChunkT> with_state(std::shared_ptr<ChunkStateT> other_state) const;
};

/**
 * Process a text character-by-character and decide which Chunk most likely has generated it.
 */
template<typename Chunk>
class Parser
{
public:
    //typedef typename StartingChunkStateT::ChunkT Chunk;
    typedef ChunkState<Chunk> ChunkStateT;
    typedef Transition<Chunk> TransitionT;

private:
    typename Containers<TransitionT>::list states;

public:
    /**
     * Construct a Parser from the initial set of states with their probabilities
     */
    Parser(typename Containers<TransitionT>::list initial_states);

    /**
     * Construct a Parser with a given initial state
     */
    Parser(typename ChunkStateT::Ptr initial_state);

    /**
     * Process the next character of the text
     */
    void next_char(char chr);

    /**
     * Process the end-of-text and return the chunk which has most likely generated the whole text
     */
    Chunk finish();

    /**
     * Return the list of states and their posterior probabilities
     */
    typename Containers<TransitionT>::list get_states();
};

/**
 * Parse an input stream starting with a given state
 */
template<typename StartingChunkStateT>
typename StartingChunkStateT::ChunkT parse(std::istream& strm, std::shared_ptr<StartingChunkStateT> state);

// Basic chunks

class Void
{};

/**
 * A ChunkState which generates an empty string and returns the given chunk as the generating Chunk
 */
template<typename Chunk>
typename ChunkState<Chunk>::Ptr empty_chunk(Chunk chunk);

/**
 * A ChunkState which generates an arbitrary character and returns that character as the generating Chunk
 */
ChunkState<char>::Ptr a_character();

/**
 * A ChunkState which generates a sequence of random characters of random length; returns Void as the generating Chunk
 */
ChunkState<Void>::Ptr garbage(double probability_finish);

/**
 * A ChunkState which generates a sequence of random characters of random length; returns Void as the generating Chunk
 */
ChunkState<Void>::Ptr garbage();

/**
 * Generates variable-length sequence of white-space characters; returns Void as the generating Chunk
 */
ChunkState<Void>::Ptr whitespace(double probability_finish);

/**
 * Generates variable-length sequence of white-space characters; returns Void as the generating Chunk
 */
ChunkState<Void>::Ptr whitespace();

/**
 * A ChunkState which generates a fixed string, and returns that string as the generating Chunk
 */
ChunkState<std::string>::Ptr literal(std::string value);

/**
 * A ChunkState which generates a non-negative decimal integer
 */
ChunkState<int>::Ptr a_positive_integer();

/**
 * A ChunkState which generates a decimal integer
 */
ChunkState<int>::Ptr an_integer();

/**
 * A ChunkState which generates a decimal integer or fraction
 *
 * @param dot_is_optional if false, the decimal dot must be present, e.g. "15." is ok, but "15" is not
 * @param probability_scientific if positive, scientific notation is allowed, e.g. "5.3e+3"
 */
ChunkState<double>::Ptr a_floating_point_number(bool dot_is_optional, double probability_scientific);

/**
 * A ChunkState which generates the same text as a given other ChunkState, but applies a transformation (function)
 * to the generator Chunk
 */
template<typename Chunk2, typename ChunkStateT1, typename Transformation>
typename ChunkState<Chunk2>::Ptr transform(std::shared_ptr<ChunkStateT1> state, Transformation transformation);

/**
 * A ChunkState which generates one out of a list of chunks, given by their initial ChunkState's
 */
template<typename ListType>
typename ChunkState<typename ListType::value_type::ChunkT>::Ptr either(/*Containers<Transition<ChunkT>>::list*/ ListType alternatives);

// Continuation API

/**
 * A chunk type which knows what kind of chunk to read next
 *
 * (TBC stands for "to be continued")
 */
template<typename NextChunk>
class ChunkTBC
{
public:
    typedef NextChunk NextChunkT;
    typedef Transition<NextChunkT> TransitionT;

    virtual const typename Containers<Transition<NextChunk>>::list next() const = 0;

    virtual double probabilityFinished() const;
};

/**
 * Given a ChunkState whose Chunk is an instance of ChunkTBC, creates a ChunkState which generates
 * this chunk followed by any continuations.
 *
 * The resultant chunk is expected to be of type LastChunk; it is a runtime error if the given ChunkTBC
 * with its continuations has a positive probability to generate a chunk of a different type.
 */
template<typename LastChunk, typename FirstChunkState>
typename ChunkState<LastChunk>::Ptr continuation(std::shared_ptr<FirstChunkState> start_state);

/**
 * Combines a ChunkState that generates a first chunk, and a functor which maps the first chunk into a
 * ChunkState to read the rest and return the ResultChunk
 */
template<typename ResultChunk, typename FirstChunkState, typename Functor>
typename ChunkState<ResultChunk>::Ptr follow(std::shared_ptr<FirstChunkState> first_state, Functor follower_generator);

/**
 * A sequence of two chunks where the result is the application of chunk1 (which should be a functor) to chunk2
 */
template<typename Result, typename ChunkState1, typename ChunkState2>
typename ChunkState<Result>::Ptr applyFirstToSecond(std::shared_ptr<ChunkState1> state1, std::shared_ptr<ChunkState2> state2);

// List template
namespace list_template
{
    /**
     * A fixed-length sequence of elements of different types
     */
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
    class StateToChunkConverter;

    template<typename StateList>
    class StatesToChunksConverter;
}

/**
 * A ChunkState which generates a sequence of chunks given by a ListT of their initial states. The generator Chunk is a ListT of the individual chunks.
 */
template<typename StateList>
typename ChunkState<typename sequence_impl::StatesToChunksConverter<StateList>::Chunks>::Ptr sequence(StateList states);

/**
 * A ChunkState which generates a sequence of chunks given by a ListT of their initial states. The generator Chunk is the index'th generated chunk.
 */
template<int index, typename List>
typename ChunkState<typename sequence_impl::StateToChunkConverter<typename list_template::impl::ListT_Getter<List, index>::TypeAt>::ChunkT>::Ptr
one_in_sequence(List list);

/**
 * A ChunkState which generates multiple repetitions of a given chunk.
 *
 * @param element The initial ChunkState for the chunk to be repeated.
 * @param min_length the minimal number of repetitions, zero or more.
 * @param max_length the maximal number of repetitions, or -1 if the number is unbounded.
 * @param initial_part will be prepended to the final Chunk
 */
template<typename ChunkStateT>
typename ChunkState<typename Containers<typename ChunkStateT::ChunkT>::list>::Ptr
star(std::shared_ptr<ChunkStateT> element, int min_length, int max_length, typename Containers<typename ChunkStateT::ChunkT>::list initial_part);

/**
 * A ChunkState which generates multiple repetitions of a given chunk.
 *
 * @param element The initial ChunkState for the chunk to be repeated.
 * @param min_length the minimal number of repetitions, zero or more.
 * @param max_length the maximal number of repetitions, or -1 if the number is unbounded.
 */
template<typename ChunkStateT>
typename ChunkState<typename Containers<typename ChunkStateT::ChunkT>::list>::Ptr star(std::shared_ptr<ChunkStateT> element, int min_length, int max_length);

// Implementation of Transition

template<typename Chunk>
Transition<Chunk>::Transition(typename ChunkState<ChunkT>::Ptr state, double prob)
:state(state), prob(prob)
{}

template<typename Chunk>
Transition<Chunk> Transition<Chunk>::operator * (double prob_multiplier) const
{
    return Transition<Chunk>(state, prob * prob_multiplier);
}

template<typename Chunk>
Transition<Chunk> Transition<Chunk>::operator / (double prob_multiplier) const
{
    return Transition<Chunk>(state, prob / prob_multiplier);
}

template<typename Chunk>
Transition<Chunk>& Transition<Chunk>::operator *= (double prob_multiplier)
{
    prob *= prob_multiplier;
    return *this;
}

template<typename Chunk>
Transition<Chunk>& Transition<Chunk>::operator /= (double prob_multiplier)
{
    prob /= prob_multiplier;
    return *this;
}

template<typename Chunk>
template<typename ChunkStateT>
Transition<typename ChunkStateT::ChunkT> Transition<Chunk>::with_state(std::shared_ptr<ChunkStateT> other_state) const
{
    return Transition<typename ChunkStateT::ChunkT>(other_state, prob);
}

// Parser implementation
template<typename Chunk>
Parser<Chunk>::Parser(typename Containers<TransitionT>::list initial_states)
: states(initial_states)
{}

template<typename Chunk>
Parser<Chunk>::Parser(typename ChunkStateT::Ptr initial_state)
: states{TransitionT(initial_state, 1)}
{}

template<typename Chunk>
void Parser<Chunk>::next_char(char chr)
{
    typename Containers<TransitionT>::list next;
    for( auto it = states.begin(); it != states.end(); ++it )
    {
        double prob = it->prob * it->state->getProbability(chr);
        if( prob > 0 )
        {
            typename Containers<TransitionT>::list trans = it->state->getTransitions(chr);
            for( auto it2 = trans.begin(); it2 != trans.end(); ++it2 )
            {
                next.push_back(TransitionT(it2->state, it2->prob * prob));
            }
        }
    }

    double totalProb = 0;
    for( auto it = next.begin(); it != next.end(); ++it )
    {
        totalProb += it->prob;
    }

    states.clear();
    for( auto it = next.begin(); it != next.end(); ++it )
    {
        states.push_back(*it / totalProb);
    }
}

template<typename Chunk>
Chunk Parser<Chunk>::finish()
{
    typename Containers<TransitionT>::list next;

    for( auto it = states.begin(); it != states.end(); ++it )
    {
        double prob = it->prob * it->state->getProbabilityFinished();
        if( prob > 0 )
        {
            next.push_back(TransitionT(it->state, it->prob * prob));
        }
    }

    if( next.empty() )
    {
        throw std::runtime_error("Syntax error (could not parse)");
    }
    else
    {
        double maxProb = next.front().prob;
        typename ChunkStateT::Ptr res = next.front().state;
        for( auto it = next.begin(); it != next.end(); ++it )
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

template<typename Chunk>
typename Containers<typename Parser<Chunk>::TransitionT>::list Parser<Chunk>::get_states()
{
    return states;
}

template<typename StartingChunkStateT>
typename StartingChunkStateT::ChunkT parse(std::istream& strm, std::shared_ptr<StartingChunkStateT> state)
{
    typedef typename StartingChunkStateT::ChunkT Chunk;
    typedef ChunkState<Chunk> ChunkStateT;
    typedef Transition<Chunk> TransitionT;

    Parser<Chunk> parser(state);

    while(strm)
    {
        char chr = strm.get();
        if( strm )
        {
            parser.next_char(chr);
        }
    }

    return parser.finish();
}

//////
// Basic chunks

/**
 * A ChunkState which generates an empty string and returns a Chunk given to it at construction
 */
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

    virtual const typename Containers<TransitionT>::list getTransitions(CharT c) const
    {
        return typename Containers<TransitionT>::list();
    }

    virtual ChunkT finish() const
    {
        return chunk;
    }
};

template<typename Chunk>
typename ChunkState<Chunk>::Ptr empty_chunk(Chunk chunk)
{
    return EmptyChunkState<Chunk>::create(chunk);
}

/**
 * A ChunkState which generates an arbitrary character and returns that character as the generating Chunk
 */
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

    virtual const Containers<Transition<char>>::list getTransitions(char c) const
    {
        Containers<Transition<char>>::list res;
        res.push_back(Transition<char>(EmptyChunkState<char>::create(c), 1));
        return res;
    }

    virtual char finish() const
    {
        throw std::logic_error("Call of ChunkState::finish() when getProbabilityFinished() == 0");
    }
};

ChunkState<char>::Ptr a_character()
{
    return SingleCharChunkState::create();
};

/**
 * Generates a sequence of random characters of random length; returns Void as the generating Chunk
 */
class GarbageChunkState: public ChunkState<Void>
{
public:
    typedef std::shared_ptr<GarbageChunkState> Ptr;

private:
    double probability_finish;

    GarbageChunkState(double probability_finish)
    : probability_finish(probability_finish)
    {}

public:
    static Ptr create(double probability_finish)
    {
        return Ptr(new GarbageChunkState(probability_finish));
    }

    virtual double getProbability(CharT c) const
    {
        return (1 - probability_finish) / 256;
    }

    virtual double getProbabilityFinished() const
    {
        return probability_finish;
    }

    virtual const Containers<TransitionT>::list getTransitions(CharT c) const
    {
        Containers<TransitionT>::list res;
        res.push_back(TransitionT(create(probability_finish), 1));
        return res;
    }

    virtual ChunkT finish() const
    {
        return Void();
    }
};

ChunkState<Void>::Ptr garbage(double probability_finish)
{
    return GarbageChunkState::create(probability_finish);
}

ChunkState<Void>::Ptr garbage()
{
    return garbage(0.01);
}

bool is_whitespace(char c)
{
    return (c == ' ') || (c == '\t');
}

int num_whitespace()
{
    return 2;
}

/**
 * Generates variable-length sequence of white-space characters; returns Void as the generating Chunk
 */
class WhitespaceChunkState: public ChunkState<Void>
{
public:
    typedef std::shared_ptr<WhitespaceChunkState> Ptr;

private:
    double probability_finish;

    WhitespaceChunkState(double probability_finish)
    : probability_finish(probability_finish)
    {}

public:
    static Ptr create(double probability_finish)
    {
        return Ptr(new WhitespaceChunkState(probability_finish));
    }

    virtual double getProbability(CharT c) const
    {
        if( is_whitespace(c) )
        {
            return (1 - probability_finish) / num_whitespace();
        }
        else
        {
            return 0;
        }
    }

    virtual double getProbabilityFinished() const
    {
        return probability_finish;
    }

    virtual const Containers<TransitionT>::list getTransitions(CharT c) const
    {
        Containers<TransitionT>::list res;
        if( is_whitespace(c) )
        {
            res.push_back(TransitionT(WhitespaceChunkState::create(probability_finish), 1));
        }
        return res;
    }

    virtual ChunkT finish() const
    {
        return Void();
    }
};

/**
 * Generates variable-length sequence of white-space characters; returns Void as the generating Chunk
 */
class WhitespaceChunkFirstState: public ChunkState<Void>
{
public:
    typedef std::shared_ptr<WhitespaceChunkFirstState> Ptr;

private:
    double probability_finish;

    WhitespaceChunkFirstState(double probability_finish)
    : probability_finish(probability_finish)
    {}

public:
    static Ptr create(double probability_finish)
    {
        return Ptr(new WhitespaceChunkFirstState(probability_finish));
    }

    virtual double getProbability(CharT c) const
    {
        if( is_whitespace(c) )
        {
            return 1.0 / num_whitespace();
        }
        else
        {
            return 0;
        }
    }

    virtual double getProbabilityFinished() const
    {
        return 0;
    }

    virtual const Containers<TransitionT>::list getTransitions(CharT c) const
    {
        Containers<TransitionT>::list res;
        res.push_back(TransitionT(WhitespaceChunkState::create(probability_finish), 1));
        return res;
    }

    virtual ChunkT finish() const
    {
        return Void();
    }
};

ChunkState<Void>::Ptr whitespace(double probability_finish)
{
    return WhitespaceChunkFirstState::create(probability_finish);
}

ChunkState<Void>::Ptr whitespace()
{
    return whitespace(0.1);
}

/**
 * A ChunkState which generates a fixed string, and returns that string as the generating Chunk
 */
class LiteralChunkState: public ChunkState<std::string>
{
public:
    typedef std::shared_ptr<LiteralChunkState> Ptr;

private:
    std::string literal;
    int pos;

    LiteralChunkState(std::string literal, int pos)
    : literal(literal), pos(pos)
    {
        if( (pos < 0) || (pos >= literal.length()) )
        {
            throw std::range_error("pos out of range in LiteralChunkState constructor");
        }
    }

public:
    static Ptr create(std::string literal, int pos)
    {
        return Ptr(new LiteralChunkState(literal, pos));
    }

    static Ptr create(std::string literal)
    {
        return create(literal, 0);
    }

    virtual double getProbability(CharT c) const
    {
        if( c == literal[pos] )
        {
            return 1;
        }
        else
        {
            return 0;
        }
    }

    virtual double getProbabilityFinished() const
    {
        return 0;
    }

    virtual const Containers<TransitionT>::list getTransitions(CharT c) const
    {
        Containers<TransitionT>::list res;
        if( c == literal[pos] )
        {
            if( pos + 1 < literal.length() )
            {
                res.push_back(TransitionT(create(literal, pos + 1), 1));
            }
            else
            {
                res.push_back(TransitionT(EmptyChunkState<std::string>::create(literal), 1));
            }
        }
        return res;
    }

    virtual ChunkT finish() const
    {
        throw std::logic_error("Call of ChunkState::finish() when getProbabilityFinished() == 0");
    }
};

ChunkState<std::string>::Ptr literal(std::string value)
{
    if( value.length() > 0 )
    {
        return LiteralChunkState::create(value);
    }
    else
    {
        return EmptyChunkState<std::string>::create(value);
    }
}

//////
// Compositions of chunks

/**
 * A ChunkState which generates the same text as a given other ChunkState, but applies a transformation (function)
 * to the generator Chunk
 */
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

    virtual const typename Containers<Transition<Chunk2>>::list getTransitions(char c) const
    {
        typename Containers<Transition<Chunk1>>::list base_transitions = base->getTransitions(c);
        typename Containers<Transition<Chunk2>>::list res;
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
 * A ChunkState which generates one out of a list of chunks, given by their initial ChunkState's
 */
template<typename Chunk>
class AlternativesChunkState: public ChunkState<Chunk>
{
public:
    typedef std::shared_ptr<AlternativesChunkState<Chunk>> Ptr;
    typedef typename ChunkState<Chunk>::CharT CharT;
    typedef typename ChunkState<Chunk>::ChunkT ChunkT;
    typedef typename ChunkState<Chunk>::TransitionT TransitionT;

private:
    typename Containers<Transition<ChunkT>>::list alternatives;

    AlternativesChunkState(const typename Containers<Transition<ChunkT>>::list alternatives)
    : alternatives(alternatives)
    {}

public:
    static Ptr create(const typename Containers<Transition<ChunkT>>::list alternatives)
    {
        return Ptr(new AlternativesChunkState(alternatives));
    }

    virtual double getProbability(CharT c) const
    {
        double res = 0.0;
        for( Transition<ChunkT> alternative : alternatives )
        {
            res += alternative.prob * alternative.state->getProbability(c);
        }
        return res;
    }

    virtual double getProbabilityFinished() const
    {
        double res = 0.0;
        for( Transition<ChunkT> alternative : alternatives )
        {
            res += alternative.prob * alternative.state->getProbabilityFinished();
        }
        return res;
    }

    virtual const typename Containers<TransitionT>::list getTransitions(CharT c) const
    {
        typename Containers<TransitionT>::list res;
        double total_prob = 0;
        for( Transition<ChunkT> alternative : alternatives )
        {
            double prob = alternative.state->getProbability(c);
            if( prob > 0 )
            {
                typename Containers<TransitionT>::list transitions = alternative.state->getTransitions(c);
                for( TransitionT transition : transitions )
                {
                    res.push_back(transition * alternative.prob * prob);
                }
                total_prob += prob;
            }
        }

        typename Containers<TransitionT>::list res_norm;
        for( const Transition<ChunkT>& transition : res )
        {
            res_norm.push_back(transition / total_prob);
        }
        return res_norm;
    }

    virtual ChunkT finish() const
    {
        typename Containers<ChunkT>::list res;
        for( Transition<ChunkT> alternative : alternatives )
        {
            if( alternative.state->getProbabilityFinished() > 0 )
            {
                res.push_back(alternative.state->finish());
            }
        }
        if( res.size() == 0 )
        {
            throw std::logic_error("Call of AlternativesChunkState::finish() when getProbabilityFinished() == 0");
        }
        else if( res.size() > 1 )
        {
            throw std::runtime_error("In AlternativesChunkState::finish(): more than one alternative could have generated the empty string");
        }
        else
        {
            return res.front();
        }
    }
};

template<typename ListType>
typename ChunkState<typename ListType::value_type::ChunkT>::Ptr either(/*Containers<Transition<ChunkT>>::list*/ ListType alternatives)
{
    typedef typename ListType::value_type::ChunkT ChunkT;
    return AlternativesChunkState<ChunkT>::create(alternatives);
}

// Continuation API implementation

template<typename NextChunk>
double ChunkTBC<NextChunk>::probabilityFinished() const
{
    double res = 1;
    for( auto transition : next() )
    {
        res -= transition.prob;
    }
    return res;
}

namespace continuation_impl
{
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

    template<typename LastChunk, typename CurChunk>
    class ContinuationHelper;

    /**
     * The state of the continuation of a ChunkTBC where CurChunk will not be finished before the current character.
     */
    template<typename LastChunk, typename CurChunk>
    class ContinuedChunkStateUnfinished: public ChunkState<LastChunk>
    {
    public:
        typedef std::shared_ptr<ContinuedChunkStateUnfinished<LastChunk, CurChunk>> Ptr;
        typedef typename ChunkState<LastChunk>::CharT CharT;
        typedef typename ChunkState<LastChunk>::ChunkT ChunkT;
        typedef typename ChunkState<LastChunk>::TransitionT TransitionT;

    private:
        typename ChunkState<CurChunk>::Ptr base;

        ContinuedChunkStateUnfinished(typename ChunkState<CurChunk>::Ptr base)
        : base(base)
        {}

    public:
        static Ptr create(typename ChunkState<CurChunk>::Ptr base)
        {
            return Ptr(new ContinuedChunkStateUnfinished(base));
        }

        virtual double getProbability(CharT c) const
        {
            return base->getProbability(c) / (1 - base->getProbabilityFinished());
        }

        virtual double getProbabilityFinished() const
        {
            return 0;
        }

        virtual const typename Containers<TransitionT>::list getTransitions(CharT c) const
        {
            return ContinuationHelper<LastChunk, CurChunk>::expandTransitions(base->getTransitions(c));
        }

        virtual ChunkT finish() const
        {
            throw std::runtime_error("ContinuedChunkStateUnfinished::finish() has been called");
        }
    };

    template<typename LastChunk, typename CurChunk, bool isTBC>
    class ContinuationHelperImpl;

    // The case of CurChunk is a ChunkTBC
    template<typename LastChunk, typename CurChunk>
    class ContinuationHelperImpl<LastChunk, CurChunk, true>
    {
    public:
        static constexpr double negligible = 1e-5;

        static const typename Containers<Transition<LastChunk>>::list expandTransitions(const typename Containers<Transition<CurChunk>>::list transitions)
        {
            typedef typename CurChunk::NextChunkT NextChunk;
            typename Containers<Transition<LastChunk>>::list res;
            typename Containers<Transition<NextChunk>>::list next;
            for( Transition<CurChunk> transition : transitions )
            {
                double prob_transition_finished = transition.state->getProbabilityFinished();
                if( prob_transition_finished < 1 )
                {
                    res.push_back(Transition<LastChunk>(
                        ContinuedChunkStateUnfinished<LastChunk, CurChunk>::create(transition.state),
                        transition.prob * (1 - prob_transition_finished)
                    ));
                }
                if( transition.prob * prob_transition_finished > negligible )
                {
                    CurChunk curChunk = transition.state->finish();
                    for( Transition<NextChunk> next_transition : curChunk.next() )
                    {
                        next.push_back(next_transition * transition.prob * prob_transition_finished);
                    }
                    if( curChunk.probabilityFinished() > 0 )
                    {
                        res.push_back(Transition<LastChunk>(
                            EmptyChunkState<LastChunk>::create(TypeCheckHelper<LastChunk, CurChunk>::check(curChunk)),
                            transition.prob * prob_transition_finished * curChunk.probabilityFinished()
                        ));
                    }
                }
            }
            if( next.size() > 0 )
            {
                for( Transition<LastChunk> transition : ContinuationHelper<LastChunk, NextChunk>::expandTransitions(next) )
                {
                    res.push_back(transition);
                }
            }
            return res;
        }
    };

    // The case of CurChunk not an instance of ChunkTBC, in which case it must also be the LastChunk
    template<typename LastChunk>
    class ContinuationHelperImpl<LastChunk, LastChunk, false>
    {
    public:
        static const typename Containers<Transition<LastChunk>>::list expandTransitions(const typename Containers<Transition<LastChunk>>::list transitions)
        {
            return transitions;
        }
    };

    // Using SFINAE to test if a given Chunk derives from ChunkTBC
    template<typename Chunk>
    class IsTBC
    {
    public:
        struct No
        {
            char a;
            char b;
        };
        template<typename NextChunk>
        static char test(ChunkTBC<NextChunk> *);
        static No test(...);
        static const bool value = sizeof(test(static_cast<Chunk*>(0))) == sizeof(char);
    };

    template<typename LastChunk, typename CurChunk>
    class ContinuationHelper
    {
    public:
        static const typename Containers<Transition<LastChunk>>::list expandTransitions(typename Containers<Transition<CurChunk>>::list transitions)
        {
            return ContinuationHelperImpl<LastChunk, CurChunk, IsTBC<CurChunk>::value>::expandTransitions(transitions);
        }

        static const typename Containers<Transition<LastChunk>>::list expandTransition(Transition<CurChunk> transition)
        {
            typename Containers<Transition<CurChunk>>::list transition_list;
            transition_list.push_back(transition);
            return expandTransitions(transition_list);
        }

        static const typename Containers<Transition<LastChunk>>::list expandState(typename ChunkState<CurChunk>::Ptr state)
        {
            return expandTransition(Transition<CurChunk>(state, 1));
        }
    };
}

template<typename LastChunk, typename FirstChunkState>
typename ChunkState<LastChunk>::Ptr continuation(std::shared_ptr<FirstChunkState> start_state)
{
    typedef typename FirstChunkState::ChunkT FirstChunk;
    return AlternativesChunkState<LastChunk>::create(continuation_impl::ContinuationHelper<LastChunk, FirstChunk>::expandState(start_state));
}

// This is deprecated: we can now use non-TBC chunks as last chunks in continuations
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

        virtual const typename Containers<Transition<NextChunkT>>::list next() const
        {
            return typename Containers<Transition<NextChunkT>>::list();
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

template<typename ResultChunk, typename FirstChunk, typename Functor>
class FollowerImpl: public ChunkTBC<ResultChunk>
{
private:
    Functor follower_generator;
    FirstChunk first_chunk;

public:
    FollowerImpl(Functor follower_generator, FirstChunk first_chunk)
    : follower_generator(follower_generator), first_chunk(first_chunk)
    {}

    class Builder
    {
    private:
        Functor follower_generator;

    public:
        Builder(Functor follower_generator)
        : follower_generator(follower_generator)
        {}

        FollowerImpl<ResultChunk, FirstChunk, Functor> operator() (FirstChunk chunk) const
        {
            return FollowerImpl<ResultChunk, FirstChunk, Functor>(follower_generator, chunk);
        }
    };

    virtual const typename Containers<Transition<ResultChunk>>::list next() const
    {
        typename Containers<Transition<ResultChunk>>::list res;
        res.push_back(Transition<ResultChunk>(follower_generator(first_chunk), 1));
        return res;
    }
};

template<typename ResultChunk, typename FirstChunkState, typename Functor>
typename ChunkState<ResultChunk>::Ptr follow(std::shared_ptr<FirstChunkState> first_state, Functor follower_generator)
{
    typedef FollowerImpl<ResultChunk, typename FirstChunkState::ChunkT, Functor> FollowerImplT;
    return continuation<ResultChunk>(transform<FollowerImplT>(first_state, typename FollowerImplT::Builder(follower_generator)));
}

//////
// Basic chunks based on ChunkTBC


template<typename Result, typename Chunk1, typename Chunk2>
class Applier: public ChunkTBC<Result>
{
private:
    Chunk1 chunk1;
    typename ChunkState<Chunk2>::Ptr state2;

public:
    Applier(Chunk1 chunk1, typename ChunkState<Chunk2>::Ptr state2)
    : chunk1(chunk1), state2(state2)
    {}

    class Builder
    {
    private:
        typename ChunkState<Chunk2>::Ptr state2;

    public:
        Builder(typename ChunkState<Chunk2>::Ptr state2)
        : state2(state2)
        {}

        Applier<Result, Chunk1, Chunk2> operator() (Chunk1 chunk1) const
        {
            return Applier<Result, Chunk1, Chunk2>(chunk1, state2);
        }
    };

    virtual const typename Containers<Transition<Result>>::list next() const
    {
        typename ChunkState<Result>::Ptr res = transform<Result>(state2, chunk1);

        typename Containers<Transition<Result>>::list res_list;
        res_list.push_back(Transition<Result>(res, 1.0));
        return res_list;
    }
};

template<typename Result, typename ChunkState1, typename ChunkState2>
typename ChunkState<Result>::Ptr applyFirstToSecond(std::shared_ptr<ChunkState1> state1, std::shared_ptr<ChunkState2> state2)
{
    typedef typename ChunkState1::ChunkT Chunk1;
    typedef typename ChunkState2::ChunkT Chunk2;
    return continuation<Result>(transform<Applier<Result, Chunk1, Chunk2>>(state1, typename Applier<Result, Chunk1, Chunk2>::Builder(state2)));
}

namespace number_impl
{
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

        friend class APositiveIntegerFirstChunkState;
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

        virtual const Containers<TransitionT>::list getTransitions(CharT c) const
        {
            if( (c >= '0') && (c <= '9') )
            {
                Containers<TransitionT>::list res;
                res.push_back(TransitionT(create(prob_finished, prefix * 10 + (c - '0')), 1));
                return res;
            }
            else
            {
                //throw std::logic_error(std::string("A call to APositiveInteger::getTransitions(") + c + ")");
                return Containers<TransitionT>::list();
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

        virtual const Containers<TransitionT>::list getTransitions(CharT c) const
        {
            Containers<TransitionT>::list res;
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

    class SignChunk
    {
    private:
        bool negative;

    public:
        typedef int NextChunkT;

        SignChunk(bool negative)
        : negative(negative)
        {}

        template<typename Number>
        Number operator()(Number value) const
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

    class SignChunkState: public ChunkState<SignChunk>
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
            else if( c == '+' )
            {
                return 0.25;
            }
            else
            {
                return 0;
            }
        }

        virtual double getProbabilityFinished() const
        {
            return 0.25;
        }

        virtual const Containers<TransitionT>::list getTransitions(CharT c) const
        {
            Containers<TransitionT>::list res;
            if( c == '-' )
            {
                res.push_back(TransitionT(EmptyChunkState<SignChunk>::create(SignChunk(true)), 1));
            }
            else if( c == '+' )
            {
                res.push_back(TransitionT(EmptyChunkState<SignChunk>::create(SignChunk(false)), 1));
            }
            return res;
        }

        virtual SignChunk finish() const
        {
            return SignChunk(false);
        }
    };

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

        virtual const Containers<TransitionT>::list getTransitions(CharT c) const
        {
            Containers<TransitionT>::list res;
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

        virtual const Containers<TransitionT>::list getTransitions(CharT c) const
        {
            Containers<TransitionT>::list res;
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

    class FloatGenerator
    {
    private:
        bool dot_is_optional;

    public:
        FloatGenerator(bool dot_is_optional)
        : dot_is_optional(dot_is_optional)
        {}

        ChunkState<double>::Ptr operator()(int integer_part) const
        {
            return FloatingPointChunkState::create(integer_part, dot_is_optional);
        }
    };

    class ScientificPartState: public ChunkState<int>
    {
    public:
        typedef std::shared_ptr<ScientificPartState> Ptr;

    private:
        double probability_scientific;

        ScientificPartState(double probability_scientific)
        : probability_scientific(probability_scientific)
        {}

    public:
        static Ptr create(double probability_scientific)
        {
            return Ptr(new ScientificPartState(probability_scientific));
        }

        virtual double getProbability(CharT c) const
        {
            if( (c == 'e') || (c == 'E') )
            {
                return probability_scientific / 2;
            }
            else
            {
                return 0;
            }
        }

        virtual double getProbabilityFinished() const
        {
            return 1 - probability_scientific;
        }

        virtual const Containers<TransitionT>::list getTransitions(CharT c) const
        {
            Containers<TransitionT>::list res;
            if( probability_scientific > 0 )
            {
                res.push_back(TransitionT(an_integer(), 1));
            }
            return res;
        }

        virtual int finish() const
        {
            return 0;
        }
    };

    class WantScientificPart: public ChunkTBC<double>
    {
    private:
        double base;
        double probability_scientific;

    public:
        WantScientificPart(double base, double probability_scientific)
        : base(base), probability_scientific(probability_scientific)
        {}

        class Builder
        {
        private:
            double probability_scientific;

        public:
            Builder(double probability_scientific)
            : probability_scientific(probability_scientific)
            {}

            WantScientificPart operator()(double base) const
            {
                return WantScientificPart(base, probability_scientific);
            }
        };

        double operator()(int order) const
        {
            return base * std::pow(10, order);
        }

        virtual const Containers<TransitionT>::list next() const
        {
            Containers<TransitionT>::list res;
            res.push_back(TransitionT(transform<double>(ScientificPartState::create(probability_scientific), *this), 1));
            return res;
        }
    };
}

ChunkState<int>::Ptr a_positive_integer()
{
    return number_impl::APositiveIntegerFirstChunkState::create();
}

ChunkState<int>::Ptr an_integer()
{
    return applyFirstToSecond<int>(number_impl::SignChunkState::create(), a_positive_integer());
}

ChunkState<double>::Ptr a_floating_point_number(bool dot_is_optional, double probability_scientific)
{
    //ChunkState<double>::Ptr positive_number = continuation<double>(transform<FirstChunkOfFloat>(a_positive_integer(), FirstChunkOfFloat::Builder(dot_is_optional)));
    ChunkState<double>::Ptr positive_number = follow<double>(a_positive_integer(), number_impl::FloatGenerator(dot_is_optional));
    ChunkState<double>::Ptr positive_number_scientific = continuation<double>(transform<number_impl::WantScientificPart>(
        positive_number,
        number_impl::WantScientificPart::Builder(probability_scientific)
    ));
    return applyFirstToSecond<double>(number_impl::SignChunkState::create(), positive_number_scientific);
}

//////
// Implementation of sequence of chunks

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

        virtual const typename Containers<Transition<NextChunk>>::list next() const
        {
            typename Containers<Transition<NextChunk>>::list res;
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

        virtual const typename Containers<Transition<NextChunk>>::list next() const
        {
            typename Containers<Transition<NextChunk>>::list res;
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

    template<int index, typename List>
    typename list_template::impl::ListT_Getter<List, index>::TypeAt take_one(List list)
    {
        return list.template at<index>();
    }
}

template<typename StateList>
typename ChunkState<typename sequence_impl::StatesToChunksConverter<StateList>::Chunks>::Ptr sequence(StateList states)
{
    typedef typename sequence_impl::StatesToChunksConverter<StateList>::Chunks ParsedList;
    typedef sequence_impl::ListChunk<ParsedList, Nil> LastChunk;
    typedef sequence_impl::ListChunk<Nil, StateList> FirstChunk;

    typename ChunkState<LastChunk>::Ptr res = continuation<LastChunk>(transform<typename FirstChunk::NextChunkT>(states.head, FirstChunk(Nil(), states)));
    return transform<ParsedList>(res, sequence_impl::extractor<ParsedList>);
}

template<int index, typename List>
typename ChunkState<typename sequence_impl::StateToChunkConverter<typename list_template::impl::ListT_Getter<List, index>::TypeAt>::ChunkT>::Ptr
one_in_sequence(List list)
{
    typedef
        typename ChunkState<typename sequence_impl::StateToChunkConverter<typename list_template::impl::ListT_Getter<List, index>::TypeAt>::ChunkT>::Ptr
        ResultT;
    return transform<typename sequence_impl::StateToChunkConverter<ResultT>::ChunkT>(
        sequence(list),
        sequence_impl::take_one<index, typename sequence_impl::StatesToChunksConverter<List>::Chunks>
    );
}

template<typename BaseChunk>
class StarChunk: public ChunkTBC<StarChunk<BaseChunk>>
{
public:
    typedef StarChunk<BaseChunk> NextChunkT;
    typedef Transition<NextChunkT> TransitionT;

private:
    typename ChunkState<BaseChunk>::Ptr base_element;
    typename Containers<BaseChunk>::list data;
    int min_length;
    int max_length;

public:
    StarChunk(typename ChunkState<BaseChunk>::Ptr start_state, int min_length, int max_length, typename Containers<BaseChunk>::list data)
    : base_element(start_state), data(data), min_length(min_length), max_length(max_length)
    {}

    StarChunk(typename ChunkState<BaseChunk>::Ptr start_state, int min_length, int max_length)
    : base_element(start_state), min_length(min_length), max_length(max_length)
    {}

    /**
     * Convert a newly read BaseChunk into the next StartChunk
     */
    StarChunk<BaseChunk> operator() (BaseChunk element) const
    {
        typename Containers<BaseChunk>::list next_data = data;
        next_data.push_back(element);
        return StarChunk<BaseChunk>(base_element, min_length, max_length, next_data);
    }

    virtual const typename Containers<TransitionT>::list next() const
    {
        typename Containers<TransitionT>::list res;
        double prob;
        if( data.size() < min_length )
        {
            prob = 1;
        }
        else if( (max_length > 0) && (data.size() >= max_length) )
        {
            prob = 0;
        }
        else
        {
            prob = 0.9;
        }
        if( prob > 0 )
        {
            res.push_back(TransitionT(transform<StarChunk<BaseChunk>>(base_element, *this), prob));
        }
        return res;
    }

    static typename Containers<BaseChunk>::list unwrap(const StarChunk<BaseChunk>& star)
    {
        return star.data;
    }
};

template<typename ChunkStateT>
typename ChunkState<typename Containers<typename ChunkStateT::ChunkT>::list>::Ptr
star(std::shared_ptr<ChunkStateT> element, int min_length, int max_length, typename Containers<typename ChunkStateT::ChunkT>::list initial_part)
{
    typedef typename ChunkStateT::ChunkT BaseChunk;
    typedef StarChunk<BaseChunk> StarChunkT;
    typedef typename Containers<BaseChunk>::list ResultT;
    auto wrapper = StarChunkT(element, min_length, max_length, initial_part);
    auto unwrapper = StarChunkT::unwrap;
    return transform<ResultT>(continuation<StarChunkT>(transform<StarChunkT>(element, wrapper)), unwrapper);
}

template<typename ChunkStateT>
typename ChunkState<typename Containers<typename ChunkStateT::ChunkT>::list>::Ptr star(std::shared_ptr<ChunkStateT> element, int min_length, int max_length)
{
    return star(element, min_length, max_length, typename Containers<typename ChunkStateT::ChunkT>::list());
}

#endif
