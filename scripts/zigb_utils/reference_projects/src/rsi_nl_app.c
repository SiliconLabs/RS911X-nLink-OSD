/**
 * @file      rsi_nl_app.c
 * @version   1.6
 * @date      2013-June-12
 *
 * Copyright(C) Redpine Signals 2013
 * All rights reserved by Redpine Signals.
 *
 * @section License
 * This program should be used on your own responsibility.
 * Redpine Signals assumes no responsibility for any losses
 * incurred by customers or third parties arising from the use of this file.
 *
 * @brief SPI, NETLINK socket init, related functions and necessary functions
 * to read the response from kernel and to handle that response
 *
 * @section Description
 * This file contains the following functions:
 *     rsi_create_nl_socket
 *     rsi_sendto_fd
 *     rsi_get_family_id
 *     rsi_nl_socket_init
 *     rsi_enqueue_to_rcv_q
 *     rsi_dequeue_from_rcv_q
 *     RecvThreadBody
 *     rsi_parse_response
 *     rsi_alloc_and_init_cmdbuff
 *     rsi_fill_genl_nl_hdrs_for_cmd
 *     rsi_send_usr_cmd
 */

/**
 * Includes
 */
#include "rsi_nl_app.h"
#include "rsi_zigb_app.h"
#include "rsi_zigb_types.h"
#include "platform_specific.h"
#include <sys/socket.h>

/**
 * Function Prototypes
 */
static int32_t rsi_create_nl_socket(int32_t protocol, int32_t groups);

/**
 * Global Variables
 */
#ifdef BT_ENABLE
extern RSI_BT_APP_CB rsi_bt_AppControlBlock;
#elif WLAN_ENABLE
extern rsi_app_cb_t rsi_app_cb;
#elif  ZB_ENABLE
extern rsi_zigb_app_cb_t rsi_zigb_app_cb;
#endif

#define WLAN_PORT_ID  0x1111 
#define ZB_PORT_ID    0x2222
#define BT_PORT_ID    0x3333

#ifdef ENABLE_WLAN_NL
#define PORT_ID    WLAN_PORT_ID
#elif ENABLE_ZB_NL
#define PORT_ID    ZB_PORT_ID
#elif ENABLE_BT_NL
#define PORT_ID    BT_PORT_ID
#endif

#define GENL_FAMILY_NAME "Obx-ZIGBgenl"
/*=================================================*/
/**
 * @fn          static int32_t rsi_create_nl_socket(int32_t protocol, int32_t groups)
 * @brief       Creates a raw netlink socket and bind
 * @param[in]   int32_t protocol
 * @param[in]   int32_t groups
 * @param[out]  none
 * @return      errCode
 *              -1 = ERROR
 *              0  = SUCCESS
 * @section description
 * This API is used to create a netlink socket and to bind the same.
 */
static int32_t rsi_create_nl_socket(int32_t protocol, int32_t groups)
{
  int32_t fd;
  struct sockaddr_nl local;

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nrsi_create_nl_socket\n");
#endif
  fd = socket(AF_NETLINK, SOCK_RAW, protocol);
  if (fd < 0)
  {
    perror("socket");
    return -1;
  }

  memset(&local, 0, sizeof(local));
  local.nl_family = AF_NETLINK;
  local.nl_groups = groups;
  local.nl_pid    = PORT_ID;
  if (bind(fd, (struct sockaddr *) &local, sizeof(local)) < 0)
    goto error;

  return fd;

error:
  close(fd);
  return -1;
}


/*=================================================*/
/**
 * @fn          int32_t rsi_sendto_fd(int32_t s, const uint8_t *buf, int32_t bufLen)
 * @brief       Sends the netlink message to Kernel
 * @param[in]   int32_t s, socket descriptor
 * @param[in]   const uint8_t *buf, message buffer to send
 * @param[in]   int32_t bufLen, Length of message buffer
 * @param[out]  none
 * @return      errCode
 *              -1 = ERROR
 *              0  = SUCCESS
 * @section description
 * This API is used to send the netlink message to Kernel.
 */
int32_t rsi_sendto_fd(int32_t s, const uint8_t *buf, int32_t bufLen)
{
  struct sockaddr_nl nladdr;
  int32_t r;

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nrsi_sendto_fd:\n");
#endif
  memset(&nladdr, 0, sizeof(nladdr));
  nladdr.nl_family = AF_NETLINK;

  while ((r = sendto(s, buf, bufLen, 0, (struct sockaddr *) &nladdr,
                     sizeof(nladdr))) < bufLen)
  {
    if (r > 0)
    {
      buf += r;
      bufLen -= r;
    }
    else if (errno != EAGAIN)
      return -1;
  }

  return 0;
}



/*=================================================*/
/**
 * @fn          int32_t rsi_get_family_id(int32_t sd)
 * @brief       To get the family ID
 * @param[in]   int32_t sd, netlink socket descriptor
 * @param[out]  none
 * @return      errCode
 *              -1 = ERROR
 *              0  = SUCCESS
 * @section description
 * This API is used to probe the controller in genetlink
 * to find the family id for the `GENL_FAMILY_NAME' family.
 */
int32_t rsi_get_family_id(int32_t sd)
{
  rsi_nlPkt_t *family_req, *ans;
  int32_t id = 0;
  struct nlattr *na;
  int32_t rep_len;
  uint8_t *req_buff = NULL, *rsp_buff = NULL;
  uint8_t family_name[25] = GENL_FAMILY_NAME;

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nrsi_get_family_id:\n");
#endif
  req_buff = rsi_malloc(NLMSG_LENGTH(GENL_HDRLEN) +
                        NLMSG_ALIGN(strlen((char*)family_name) + 1 + NLA_HDRLEN));

  family_req = (rsi_nlPkt_t *)req_buff;

  /* Get family name */
  family_req->n.nlmsg_type = GENL_ID_CTRL;
  family_req->n.nlmsg_flags = NLM_F_REQUEST;
  family_req->n.nlmsg_seq = 0;
  family_req->n.nlmsg_pid = getpid();
  family_req->n.nlmsg_len = NLMSG_LENGTH(GENL_HDRLEN);
  family_req->g.cmd = CTRL_CMD_GETFAMILY;
  family_req->g.version = 0x1;

  na = (struct nlattr *) GENLMSG_DATA(req_buff);
  na->nla_type = CTRL_ATTR_FAMILY_NAME;
  /*------change here--------*/
  na->nla_len = strlen((char*)family_name) + 1 + NLA_HDRLEN;
  strcpy((char*)NLA_DATA(na),(char*) family_name);

  family_req->n.nlmsg_len += NLMSG_ALIGN(na->nla_len);

  if (rsi_sendto_fd(sd, (req_buff), family_req->n.nlmsg_len) < 0)
    {
        rsi_free(req_buff);
	    return -1;
    }
  else
    rsi_free(req_buff);

  rsp_buff = rsi_malloc(MAX_RCV_SIZE);

  ans = (rsi_nlPkt_t *)rsp_buff;

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL13,"waiting to recv family_id\n");
#endif
  rep_len = recv(sd, ans, MAX_RCV_SIZE, 0);
  if (rep_len < 0)
  {
    rsi_free(rsp_buff);
    perror("recv");
#if ((defined LINUX_PLATFORM) && (defined RSI_DEBUG_PRINT))
    fprintf(stderr,"rsi_get_family_id ERROR NUMBER = %d",errno);
#endif
    return -1;
  }

  /* Validate response message */
  if (!NLMSG_OK((&ans->n), rep_len))
  {
    rsi_free(rsp_buff);
#if ((defined LINUX_PLATFORM) && (defined RSI_DEBUG_PRINT))
    fprintf(stderr, "invalid reply message\n");
#endif
    return -1;
  }

  if (ans->n.nlmsg_type == NLMSG_ERROR)
  { /* error */
    rsi_free(rsp_buff);
#if ((defined LINUX_PLATFORM) && (defined RSI_DEBUG_PRINT))
    fprintf(stderr, "received error\n");
#endif
    return -1;
  }

  na = (struct nlattr *) GENLMSG_DATA(ans);
  na = (struct nlattr *) ((char *) na + NLA_ALIGN(na->nla_len));
  if (na->nla_type == CTRL_ATTR_FAMILY_ID)
  {
    id = *(__u16 *) NLA_DATA(na);
  }

  rsi_free(rsp_buff);

  return id;
}

/*=================================================*/
/**
 * @fn          int32_t rsi_nl_socket_init(void)
 * @brief       Initialises the netlink socket
 * @param[in]   none
 * @param[out]  none
 * @return      ErrCode
 *              -1 = ERROR
 *              0  = SUCCESS
 * @section description
 * This API is used to create a netlink socket and to bind the same.
 */
int32_t rsi_nl_socket_init(void)
{
  rsi_linux_app_cb_t *linux_app_cbPtr = &rsi_linux_app_cb;
  int32_t retval = 0;

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nrsi_nl_socket_init\n");
#endif
  linux_app_cbPtr->nl_sd = rsi_create_nl_socket(NETLINK_GENERIC,0);

  if(linux_app_cbPtr->nl_sd < 0)
  {
#ifdef RSI_DEBUG_PRINT
    RSI_DPRINT(RSI_PL13,"create failure\n");
#endif
    return 0;
  }

  linux_app_cbPtr->family_id = rsi_get_family_id(linux_app_cbPtr->nl_sd);

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL13,"family id: %ld\n",linux_app_cbPtr->family_id);
#endif

  return retval;
}


/*=================================================*/
/**
 * @fn          void rsi_enqueue_to_rcv_q(pkt_struct_t *Pkt)
 * @brief       To enqueue the packet to receive queue
 * @param[in]   pkt_struct_t *Pkt
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to enqueue the received packet from kernel
 * to receive queue.
 */
void rsi_enqueue_to_rcv_q(pkt_struct_t *Pkt)
{
  pkt_queue_t *rcv_q = &rsi_linux_app_cb.rcv_queue;

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nrsi_enqueue_to_rcv_q:\n");
#endif
  if(rcv_q->tail == NULL)
  {
    rcv_q->head = Pkt;
  }
  else
  {
    rcv_q->tail->next  = Pkt;
  }
  rcv_q->tail = Pkt;
  rcv_q->tail->next = NULL;
  rcv_q->pending_pkt_count++;
#ifdef BT_ENABLE
  printf("Receive packet is pending...\n ");
  rsi_bt_AppControlBlock.PacketPending = RSI_TRUE;
#elif WLAN_ENABLE
  rsi_app_cb.pkt_pending = RSI_TRUE;
#elif ZB_ENABLE
  rsi_zigb_app_cb.pkt_pending = RSI_TRUE;
#endif  
  
  return;
}


/*=================================================*/
/**
 * @fn          pkt_struct_t *rsi_dequeue_from_rcv_q(void)
 * @brief       To dequeue the packet to receive queue
 * @param[in]   none
 * @param[out]  none
 * @return      pkt_struct_t *Pkt, dequeued packet pointer
 * @section description
 * This API is used to dequeue the packets from receive queue
 * when packets are pending in queue.
 */
pkt_struct_t *rsi_dequeue_from_rcv_q(void)
{
  pkt_struct_t *Pkt = NULL;
  pkt_queue_t *rcv_q = &rsi_linux_app_cb.rcv_queue;

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nrsi_dequeue_from_rcv_q:\n");
#endif
  if(rcv_q->pending_pkt_count > 0)
  {
    rcv_q->pending_pkt_count--;
    Pkt = rcv_q->head;
    rcv_q->head = rcv_q->head->next;
    if(rcv_q->head == NULL)
    {
      rcv_q->tail = NULL;
#ifdef ZB_ENABLE
      rsi_zigb_app_cb.pkt_pending = RSI_FALSE;
#endif
    }
  }

  return Pkt;
}


/*=================================================*/
/**
 * @fn          void &RecvThreadBody(void *arg)
 * @brief       Receive thread function
 * @param[in]   void *arg, receive thread func argument
 * @param[out]  none
 * @return      none
 * @section description
 * This is Recv thread function which will receive packets
 * from kernel over netlink socket
 */
void * RecvThreadBody(void * arg )
{
  pkt_struct_t *rcvPktPtr;
  int32_t rsp_len;

  char *s = arg;

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nRecvThreadBody:\n");
#endif
  RSI_DPRINT(RSI_PL13,"%s\n",s);
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
    if(rsp_len < 0)
    {
      perror("recv");
#if ((defined LINUX_PLATFORM) && (defined RSI_DEBUG_PRINT))
      fprintf(stderr," RecvThreadBody ERROR NUMBER = %d \n",errno);
#endif
      if(errno == ENOBUFS || errno == ESPIPE)
      {
        //! Handling for No buffer space available Error
        rsi_free(rcvPktPtr);
        continue;
      }
      return NULL;
    }

    pthread_mutex_lock(&rsi_linux_app_cb.mutex1);
    rsi_enqueue_to_rcv_q(rcvPktPtr);
    pthread_mutex_unlock(&rsi_linux_app_cb.mutex1);
  }
}



/*=================================================*/
/**
 * @fn          uint8_t *rsi_alloc_and_init_cmdbuff(uint8_t *Desc,
 *                                       uint8_t *payload,
 *                                       uint16_t payload_size)
 * @brief       To allocate and initialise the command buffer.
 * @param[in]   pkt_struct_t *Pkt
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to allocate a buffer for the command to send and
 * initializing it with all the header bytes, Desc and payload filled.
 */
uint8_t *rsi_alloc_and_init_cmdbuff(uint8_t *Desc, uint8_t *payload, uint16_t payload_size)
{
  uint8_t          *cmd_buff;
  rsi_nlPkt_t    *req;
  struct nlattr  *na;

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nrsi_alloc_and_init_cmdbuff\n");
#endif
  cmd_buff = rsi_malloc(payload_size + RSI_FRAME_DESC_LEN + RSI_NL_HEAD_SIZE);
  req = (rsi_nlPkt_t *)cmd_buff;

  memcpy(cmd_buff, rsi_linux_app_cb.rsi_glbl_genl_nl_hdr, RSI_NL_HEAD_SIZE - 4);
  /*compose message*/
  na = (struct nlattr *) GENLMSG_DATA(cmd_buff);
  na->nla_type = 1; //!DOC_EXMPL_A_MSG
  na->nla_len = payload_size + RSI_FRAME_DESC_LEN + NLA_HDRLEN + 2; //!message length
  memcpy(NLA_DATA(na), Desc, RSI_FRAME_DESC_LEN);

  if(payload_size)
    memcpy(NLA_DATA(na) + RSI_FRAME_DESC_LEN, payload, payload_size);
  req->n.nlmsg_len += NLMSG_ALIGN(na->nla_len);

  return cmd_buff;
}


/*=================================================*/
/**
 * @fn          void rsi_fill_genl_nl_hdrs_for_cmd(void)
 * @brief       To fill the hdrs global array
 * @param[in]   none
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to fill the global array which will be used
 * to fill the header information for the commands to send afterward.
 */
void rsi_fill_genl_nl_hdrs_for_cmd(void)
{
  rsi_nlPkt_t    *req;

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nrsi_fill_genl_nl_hdrs_for_cmd\n");
#endif
  req = (rsi_nlPkt_t *)(rsi_linux_app_cb.rsi_glbl_genl_nl_hdr);

  /* Send command needed */
  req->n.nlmsg_len = NLMSG_LENGTH(GENL_HDRLEN);
  req->n.nlmsg_type = rsi_linux_app_cb.family_id;
  req->n.nlmsg_flags = NLM_F_REQUEST;
  req->n.nlmsg_seq = 60;
  req->n.nlmsg_pid = getpid();
  req->g.cmd = 1;

  return;
}


/*=================================================*/
/**
 * @fn          int16_t rsi_send_usr_cmd(uint8_t *buff, uint16_t bufLen)
 * @brief       To send the command to kernel through netlink socket
 * @param[in]   uint8_t *buff, pointer to command buffer
 * @param[in]   uint16_t bufLen, length of command buffer
 * @param[out]  none
 * @return      none
 * @section description
 * This API is used to send the command from user space to kernel
 * space over netlink socket.
 */
int16_t rsi_send_usr_cmd(uint8_t *buff, uint16_t bufLen)
{
  struct sockaddr_nl nladdr;
  int retval = 0;

#ifdef RSI_DEBUG_PRINT
  RSI_DPRINT(RSI_PL14,"\nrsi_send_usr_cmd\n");
#endif
  memset(&nladdr, 0, sizeof(nladdr));
  nladdr.nl_family = AF_NETLINK;
	 
  retval = sendto(rsi_linux_app_cb.nl_sd, (char *)buff, bufLen, 0,
                  (struct sockaddr *) &nladdr, sizeof(nladdr));
   return retval;
}
