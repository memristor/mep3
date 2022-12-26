/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

typedef struct _canMsg{
    uint32_t ID;
    uint8_t length;
    uint8_t data[8];
    CAN_MSG_RX_ATTRIBUTE msgAttr;
} can_t;