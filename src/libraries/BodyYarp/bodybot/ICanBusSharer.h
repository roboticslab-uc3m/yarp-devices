// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

class ICanBusSharer
{
public:
    /**
     * Destructor.
     */
    virtual ~ICanBusSharer() {}

    /**
     * Interpret a can bus message.
     * @return true/false.
     */
    virtual bool interpretMessage( can_msg * message) = 0;

};
