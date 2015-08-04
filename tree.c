/*

 * C Program to Traverse the Tree Non-Recursively

 */

#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include <shmem.h>

#define NODE_NUM  10000 
int verify_num = 0; // only for verifying
struct node
{
    int a;
    struct node *left;
    struct node *right;
};

void generate(struct node *, int);

struct node*
search(struct node *, int);

struct node*
modify(struct node*, int);

int move(struct node*, struct node*);

void delete(struct node **);
void verify(struct node*);
 
struct node head; //global
int main()
{
  int choice = 0, num, key, mype;
    struct node* flag = NULL;

    start_pes(0);
    mype = shmem_my_pe();

    srand(time(NULL));
    key = rand();
    head.a = key;
    head.left = head.right = NULL;
    //create a large enough tree in memory
    for(num = 1; num < NODE_NUM; ++num) {
      key = rand();
      generate(&head, key); 
      printf("rand %d, == %d\n", key, mype);
    }

    verify(&head);
    if(verify_num == NODE_NUM)
      printf("verify successed! PE=%d\n", mype);
    else
      printf("verify failed! %d\n", verify_num);

    scanf("%d", &key);
    flag = search(&head, key);
    if(flag != NULL)
      printf("search result: %d\n", flag->a);
    else
      printf("search failed!\n");
    /*    do

    {

        printf("\nEnter your choice:\n1. Insert\n2. Search\n3. Exit\nChoice: ");

        scanf("%d", &choice);

        switch(choice)

        {

        case 1: 

            printf("Enter element to insert: ");

            scanf("%d", &num);

            generate(&head, num);

            break;

        case 2: 

            printf("Enter key to search: ");

            scanf("%d", &key);

            flag = search(head, key);

            if (flag != NULL)

            {

                printf("Key found in tree\n");

            }

            else

            {

                printf("Key not found\n");

            }

            break;

        case 3: 

            delete(&head);

            printf("Memory Cleared\nPROGRAM TERMINATED\n");

            break;

        default: printf("Not a valid input, try again\n");

        }

	} while (choice != 3);*/

    return 0;

}

 

void generate(struct node *head, int num)
{
    struct node *temp = head, *prev = head;

    /*    if (*head == NULL)
    {
        *head = (struct node *)malloc(sizeof(struct node));
        (*head)->a = num;
        (*head)->left = (*head)->right = NULL;
    }
    else
    {*/
        while (temp != NULL)
        {
            if (num >= temp->a)
            {
                prev = temp;
                temp = temp->right;
            }
            else
            {
                prev = temp;
                temp = temp->left;
            }
        }
        temp = (struct node *)shmalloc(sizeof(struct node));
        temp->a = num;
	temp->left = temp->right = NULL;
        if (num >= prev->a)
        {
            prev->right = temp;
        }
        else
        {
            prev->left = temp;
        }
	//    }
}

 
struct node*
search(struct node *head, int key)
{
  //  stm_begin();
  //  struct node *root = stm_read(head);
    while (head != NULL)
  //I should check the head's version. do copy if necessary.
    {
        if (key > head->a)
        {
            head = head->right;
        }
        else if (key < head->a)
        {
            head = head->left;
        }
        else //key == head->a
        {
            return head;
        }
    }
  return 0;
}

/*
 * find the position for the key in the head is a (sub) tree
 * return the node that is modified
 * return 0 indicates failed because I can't add a new node now.
 */
struct node*
modify(struct node* head, int key) {
  return 0;
} 

/*
 * move 'from' to 'to'.
 * it will fail if to has no empty position for from.
 */
int move(struct node* from, struct node* to) {
  return 0;
}

void delete(struct node **head)

{

    if (*head != NULL)

    {

        if ((*head)->left)

        {

            delete(&(*head)->left);

        }

        if ((*head)->right)

        {

            delete(&(*head)->right);

        }

        free(*head);

    }

}

void verify(struct node* head) {

  if(head != NULL) {
    verify_num += 1;
    verify(head->right);
    verify(head->left);

  }
}
