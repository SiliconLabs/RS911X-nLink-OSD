#include "platform_specific.h"
#include "rsi_nl_app.h"
#include "rsi_zigb_types.h"
#include "rsi_zigb_global.h"
#include "rsi_zigb_api.h"


rsi_linux_app_cb_t   rsi_linux_app_cb;
pthread_t zigb_rcv_thd;

int32_t rsi_zigb_framework_init(void);
void * zigb_read_pkt_thread(void *arg);
int16_t rsi_frame_read(uint8_t *packet_buffer);
int32_t zigb_main(uint32_t );

/*===========================================================================
 *
 * @fn          int main()
 * @brief       Function to Initiate ZigBee App
 * @param[in]   none
 * @param[out]  none
 * @return      status
 * @section description
 * This main is used to initiate ZigBee Application
 *
 * ===========================================================================*/
  
int main(int argc, char *argv[])
{
  int32_t status = RSI_ZB_SUCCESS;        
  uint8_t skip_card_ready = 0;
  uint32_t channel = 0;
  
  status = rsi_zigb_framework_init();
#ifdef ZB_DEBUG  
  RSI_DPRINT(RSI_PL14,"Framework init done\n");
#endif
  if (status != RSI_ZB_SUCCESS) {
     RSI_DPRINT(RSI_PL14,"\n Error in Netlink socket creation \n");
     return -1;
  }
  channel = atoi(argv[1]);
  status = zigb_main(channel);
  if (status != RSI_ZB_SUCCESS) {
     RSI_DPRINT(RSI_PL1,"\n Exit from ZigBee main \n");
     return -1;
  }

  exit(0);
}

/*===========================================================================
 *
 * @fn          int32_t rsi_zigb_framework_init(void)
 * @brief       Prepares the ZigBee Specific descriptor 
 * @param[in]   buffer- buffer pointer to fill the descriptor
 * @param[out]  none
 * @return      Buffer pointer after appending descriptor
 * @section description
 * This API is used to prepare the ZigBee specific descriptor
 *
 * ===========================================================================*/
  
int32_t rsi_zigb_framework_init(void)
{
  int16_t retval = RSI_ZB_SUCCESS;

  retval = rsi_nl_socket_init();
  if (retval != RSI_ZB_SUCCESS) {
    RSI_DPRINT(RSI_PL1,"\n Netlink socket creation failed \n");
    return RSI_ZB_FAIL;
  }

#ifdef ZB_DEBUG  
  RSI_DPRINT(RSI_PL1,"NL Socket created\n");
#endif
  rsi_fill_genl_nl_hdrs_for_cmd();
#ifdef ZB_DEBUG  
  RSI_DPRINT(RSI_PL1,"NL gnl hdr filled\n");
#endif

  if(pthread_mutex_init(&rsi_linux_app_cb.mutex1, NULL) != 0) {
    RSI_DPRINT(RSI_PL1,"\n Mutex init failed...");
    return RSI_ZB_FAIL;
  }

  if(pthread_create(&zigb_rcv_thd, NULL, zigb_read_pkt_thread, 0)) {
    RSI_DPRINT(RSI_PL1,"\n Receive Thread creation failed \n");
    return RSI_ZB_FAIL;
  }

  return RSI_ZB_SUCCESS;
}

void * zigb_read_pkt_thread(void *arg)
{
  pkt_struct_t *rcvPktPtr;
  int32_t rsp_len;

  //char *s = arg;

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nRecvThreadBody:\n");
#endif
  //RSI_DPRINT(RSI_PL13,"%s\n",s);
  while(1)
  {
    rcvPktPtr = (pkt_struct_t*)rsi_malloc(RSI_ZIGB_MAX_PAYLOAD_SIZE + RSI_RXPKT_HEAD_ROOM);
    if(rcvPktPtr == NULL)
    {
#ifdef RSI_DEBUG_PRINT
      RSI_DPRINT(RSI_PL13,"Allocation failed to recv packet\n");
#endif
      return NULL;
    }
    rcvPktPtr->data = (uint8_t *)(((uint8_t *)rcvPktPtr) +  RSI_RXPKT_HEAD_ROOM);

    rsp_len = recv(rsi_linux_app_cb.nl_sd, rcvPktPtr->data , RSI_ZIGB_MAX_PAYLOAD_SIZE, 0);
#ifdef ZB_DEBUG  
  RSI_DPRINT(RSI_PL1,"recvd resp in Thread \n");
#endif
    if(rsp_len < 0)
    {
      perror("recv");
      return NULL;
    }
    pthread_mutex_lock(&rsi_linux_app_cb.mutex1);
    rsi_enqueue_to_rcv_q(rcvPktPtr);
    pthread_mutex_unlock(&rsi_linux_app_cb.mutex1);
  }
}

int16_t rsi_frame_read(uint8_t *packet_buffer)
{
  /* Variables */
  rsi_linux_app_cb_t *linux_app_cbPtr = &rsi_linux_app_cb;
  pkt_struct_t *rspPtr = NULL;

  /* Length of the data to copy */
  int16_t length = 0;

  /* Pointer to the Packet file descriptor */
  uint8_t *descPtr    = NULL;

#ifdef RSI_DEBUG_PRINT
  int i;
#endif

  /* Do actual deque from the RX queue */
  pthread_mutex_lock(&linux_app_cbPtr->mutex1);
  rspPtr = rsi_dequeue_from_rcv_q();
  pthread_mutex_unlock(&linux_app_cbPtr->mutex1);

  /* Assign pointers to appropriate addresses */
  descPtr    = rspPtr->data + RSI_NL_HEAD_SIZE;

  /* Calculate length of the packet from the first two bytes of the frame descriptor */
#ifdef RSI_LITTLE_ENDIAN
  length = *(int16_t*)descPtr & 0x0fff;
#else
  length = rsi_bytes2R_to_uint16(descPtr);
#endif
  length += RSI_FRAME_DESC_LEN;

  /* Debug: Print the length & contents of the packet */
#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL0,"RX Len of the packet: %d\n", length);
  for (i=0; i<length; i++) {
      RSI_DPRINT (RSI_PL0, "0x%x ", descPtr[i]);
      if ((i % 16 == 0) && (i != 0)) {
          RSI_DPRINT(RSI_PL0, "\n");
      }
  }
  RSI_DPRINT(RSI_PL0, "\n");
#endif

  memset(packet_buffer, 0, length);
  memcpy(packet_buffer, descPtr, length);

  rsi_free(rspPtr);

  /* Return success */
  return RSI_ZB_SUCCESS;
}
