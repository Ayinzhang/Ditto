using System;
using UnityEngine;

public class Ditto2D : MonoBehaviour
{
    public enum Type { None, Rope, SoftBody, Water }
    public Type _type = Type.None; 
    public Type type { get => _type; set { if (_type != value) { _type = value; GenerateMesh();}}}
    public int _subdiv = 5; 
    public int subdiv { get => _subdiv; set { if (_subdiv != value) { _subdiv = value; GenerateMesh();}}}
    public int k = 1000, iter = 5; public float mass = 1f, damping = 0.95f; public Vector2 force = new Vector2(0, -9.8f);
    //public bool ApproxCollision { get { return approxCollision; } set { if (value != approxCollision) ; } }

    [SerializeField] SpriteRenderer spriteRenderer; [SerializeField] MeshFilter meshFilter; 
    [SerializeField] MeshRenderer meshRenderer; [SerializeField] PolygonCollider2D polygonCollider;
    [SerializeField] Mesh mesh; [SerializeField] Sprite sprite; [SerializeField] Vector3[] verts; 
    [SerializeField] Vector2[] uv, path; [SerializeField] int[] triangles;
    [SerializeField] float dt, dt2, radius, splitx, splity;
    public Vector2[] pos, lpos, vels, fors;

    void Start()
    {
        dt = Time.fixedDeltaTime; dt2 = dt * dt;
    }

    void FixedUpdate()
    {
        float d = MathF.Pow(damping, Time.fixedDeltaTime / iter), dx = splitx * transform.localScale.x, 
            dy = splity * transform.localScale.y, idt = dt / iter;

        switch (type)
        {
            case Type.Rope:
                float a = 1f / k / dt2;
                for (int i = 0; i < pos.Length; i++)
                    pos[i] = transform.TransformPoint((path[2 * i] + path[2 * i + 1]) / 2);
                for (int i = 0; i < iter; i++)
                {
                    for (int j = 1; j < pos.Length; j++)
                    {
                        // Free Update
                        vels[j] = d * vels[j] + fors[j] / mass * idt;
                        pos[j] += vels[j] * idt; fors[j] = force;

                        // XPBD Constraint
                        Vector2 dp = pos[j - 1] - pos[j], grad = -(1 - dy / dp.magnitude) * dp / a / 2;
                        if (j == 1) pos[j] -= 2 * grad;
                        else { pos[j - 1] += grad; pos[j] -= grad; }
                        vels[j] = (pos[j] - lpos[j]) / idt; lpos[j] = pos[j];
                    }
                }
                for (int i = 0; i < pos.Length; i++)
                {
                    Vector2 dir;
                    if(i == 0) dir = (pos[i] - pos[i + 1]).normalized;
                    else if (i == pos.Length - 1) dir = (pos[i - 1] - pos[i]).normalized;
                    else dir = (pos[i - 1] - pos[i + 1]).normalized;
                    path[2 * i] = verts[2 * i] = transform.InverseTransformPoint(pos[i] - dx * Vector2.Perpendicular(dir));
                    path[2 * i + 1] = verts[2 * i + 1] = transform.InverseTransformPoint(pos[i] + dx * Vector2.Perpendicular(dir));
                }
                break;
        }

        mesh.vertices = verts; polygonCollider.SetPath(0, path);
    }

    void OnTriggerStay2D(Collider2D other)
    {
        Rigidbody2D rb = other.attachedRigidbody;
        if (rb == null) return;

        Vector2 collisionPoint = other.transform.position;

        int closestIndex1 = -1, closestIndex2 = -1;
        float minDistance1 = float.MaxValue, minDistance2 = float.MaxValue;

        for (int i = 0; i < pos.Length; i++)
        {
            float distance = Vector2.Distance(collisionPoint, pos[i]);
            if (distance < minDistance1)
            {
                minDistance2 = minDistance1; closestIndex2 = closestIndex1;
                minDistance1 = distance; closestIndex1 = i;
            }
            else if (distance < minDistance2)
            {
                minDistance2 = distance; closestIndex2 = i;
            }
        }

        if (closestIndex1 == -1 || closestIndex2 == -1) return;

        Vector2 dir1 = (pos[closestIndex1] - collisionPoint).normalized;
        Vector2 dir2 = (pos[closestIndex2] - collisionPoint).normalized;

        Vector2 v1 = dir1 * Vector2.Dot(dir1, rb.linearVelocity);
        Vector2 v2 = dir2 * Vector2.Dot(dir2, rb.linearVelocity);

        vels[closestIndex1] += v1; vels[closestIndex2] += v2;
    }


    void OnDrawGizmosSelected()
    {
        switch (type)
        {
            case Type.Rope:
                for (int i = 0; i < pos.Length; i++)
                {
                    Gizmos.color = i == 0 ? Color.red : Color.green;
                    Gizmos.DrawSphere(pos[i], radius * transform.localScale.y);
                }
                break;
        }
    }

    void GenerateMesh()
    {
        spriteRenderer = GetComponent<SpriteRenderer>();
        if (spriteRenderer != null)
        {
            sprite = spriteRenderer.sprite;
            DestroyImmediate(spriteRenderer);
        }
        meshFilter = GetComponent<MeshFilter>();
        if (meshFilter == null) meshFilter = gameObject.AddComponent<MeshFilter>();
        meshRenderer = GetComponent<MeshRenderer>();
        if (meshRenderer == null)
        { 
            meshRenderer = gameObject.AddComponent<MeshRenderer>();
            meshRenderer.material = new Material(Shader.Find("Sprites/Default"));
        }
        polygonCollider = GetComponent<PolygonCollider2D>();
        if (polygonCollider == null) polygonCollider = gameObject.AddComponent<PolygonCollider2D>();

        mesh = new Mesh { name = "Generated Mesh" };
        switch (type)
        {
            case Type.Rope:
                mass = 0.01f; polygonCollider.isTrigger = true;
                verts = new Vector3[(_subdiv + 1) * 2]; uv = new Vector2[(_subdiv + 1) * 2]; 
                path = new Vector2[(_subdiv + 1) * 2]; pos = new Vector2[_subdiv + 1];
                lpos = new Vector2[_subdiv + 1]; vels = new Vector2[_subdiv + 1];
                fors = new Vector2[_subdiv + 1]; triangles = new int[_subdiv * 6]; 
                Vector3 size = sprite != null ? sprite.bounds.size : meshRenderer.bounds.size;
                splitx = size.x / 2; splity = size.y  / _subdiv; radius = splity / 3;
                for (int i = 0; i <= _subdiv; i++)
                {
                    float y = (_subdiv - i) * splity;
                    pos[i] = lpos[i] = transform.TransformPoint(new Vector3(0, y, 0));
                    path[i * 2] = verts[i * 2] = new Vector3(-splitx, y, 0); 
                    path[i * 2 + 1] = verts[i * 2 + 1] = new Vector3(splitx, y, 0); 
                    uv[i * 2] = new Vector2(0, i / (float)_subdiv);
                    uv[i * 2 + 1] = new Vector2(1, i / (float)_subdiv);

                    if (i < _subdiv)
                    {
                        int start = i * 2, triIndex = i * 6;
                        triangles[triIndex] = start;
                        triangles[triIndex + 1] = start + 2;
                        triangles[triIndex + 2] = start + 1;
                        triangles[triIndex + 3] = start + 1;
                        triangles[triIndex + 4] = start + 2;
                        triangles[triIndex + 5] = start + 3;
                    }
                }
                break;
        }

        mesh.vertices = verts; mesh.uv = uv; mesh.triangles = triangles; meshFilter.mesh = mesh;
        polygonCollider.pathCount = 1; polygonCollider.SetPath(0, path);
    }
}
