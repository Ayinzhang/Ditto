using UnityEngine;

public class Player : MonoBehaviour
{
    Rigidbody2D rb;

    void Start()
    {
        rb = GetComponent<Rigidbody2D>();
    }

    void Update()
    {
        rb.AddForceX(Input.GetAxis("Horizontal"));
        if(Input.GetKeyDown(KeyCode.K) || Input.GetKeyDown(KeyCode.Space)) rb.AddForceY(300);
    }
}
