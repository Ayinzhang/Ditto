using UnityEditor;
using UnityEngine;

[CustomEditor(typeof(Ditto2D))]
public class Ditto2DInspector : Editor
{
    Ditto2D m_target;

    void OnEnable()
    {
        m_target = (Ditto2D)target;
    }

    public override void OnInspectorGUI()
    {
        //base.OnInspectorGUI(); 
        m_target.type = (Ditto2D.Type)EditorGUILayout.EnumPopup("Type", m_target.type);
        m_target.subdiv = EditorGUILayout.IntField("Subdivided", m_target.subdiv);
        m_target.k = EditorGUILayout.IntField("K", m_target.k);
        m_target.iter = EditorGUILayout.IntField("Iterations", m_target.iter);
        m_target.mass = EditorGUILayout.FloatField("Mass", m_target.mass);
        m_target.damping = EditorGUILayout.FloatField("Damping", m_target.damping);
        m_target.force = EditorGUILayout.Vector2Field("Force", m_target.force);
    }
}
