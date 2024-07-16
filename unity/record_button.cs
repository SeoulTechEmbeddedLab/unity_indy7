using System.IO;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.EventSystems;
using System;

public class record_button : MonoBehaviour, IPointerClickHandler
{
    public GameObject indy7_object;
    public ArticulationBody[] articulation_body_list;

    public Material button_material;

    //TEST FILE
    StreamWriter test_writer;
    string test_prefix = "Assets/test/"; string test_name = "test"; string test_suffix = ".txt";
    //string test_file = "Assets/test/test.txt";

    //RECORD PERIOD: 1ms
    public const float period = 0.001f;

    public long start_tick = 0;
    public long current_tick = 0;
    private Color color_offset = new Color(0, 0.213333f, 0.213333f);
    private Color color_offset_2 = new Color(0.250980f, 0, 0);

    // Start is called before the first frame update
    void Start()
    {
        indy7_object = GameObject.Find("indy7");
        articulation_body_list = indy7_object.GetComponentsInChildren<ArticulationBody>();

        button_material = GetComponent<Renderer>().material;
        button_material.color = Color.red + color_offset - color_offset_2;
    }
    
    public bool boolean_button = false;

    //STATIC VALUE
    public static bool static_boolean_button = false;

    // Update is called once per frame
    void Update()
    {
        //버튼이 활성화 되면 기록 시작
        if(boolean_button == true)
        {
            StartCoroutine(test_write());
        }

        //static test
        if (static_boolean_button == true)
        {
            static_boolean_button = false;
            test_init();
        }

    }

    public void test_init()
    {
        //TEST FILE
        test_name = "test_" + DateTime.Now.TimeOfDay.ToString().Replace(':', '_');
        File.Create(test_name);

        test_writer = new StreamWriter(test_prefix + test_name + test_suffix, true);

        //button_material.color = Color.red;
        button_material.color = button_material.color - color_offset + color_offset_2;

        start_tick = DateTime.Now.Ticks;


        boolean_button = true;
    }

    string test_string;

    void test_print()
    {
        test_string = (-articulation_body_list[2].transform.localRotation.eulerAngles.y * (Math.PI / 180.0)) + " "
                        + (articulation_body_list[3].transform.localRotation.eulerAngles.x * (Math.PI / 180.0) - (Math.PI / 2)) + " "
                        + (-articulation_body_list[4].transform.localRotation.eulerAngles.y * (Math.PI / 180.0)) + " "
                        + (articulation_body_list[5].transform.localRotation.eulerAngles.x * (Math.PI / 180.0)) + " "
                        + (-articulation_body_list[6].transform.localRotation.eulerAngles.x * (Math.PI / 180.0) + (Math.PI / 2)) + " "
                        + (articulation_body_list[7].transform.localRotation.eulerAngles.x * (Math.PI / 180.0)) + " ";
        print(test_string);
        boolean_button = false;
        button_material.color = Color.white;
    }

    IEnumerator test_write()
    {
        /**********************************************************************************
         *   C# explanation for ticks 
         *   address: https://learn.microsoft.com/en-us/dotnet/api/system.datetime.ticks?view=net-8.0#system-datetime-ticks
         *   
         *   Remarks
         *   A single tick represents one hundred nanoseconds or one ten-millionth of a second. 
         *   There are 10,000 ticks in a millisecond (see TicksPerMillisecond) and 10 million ticks in a second.
         *   
         *   10 tick = 1 us 
         *   10000 tick = 1 ms 
         **********************************************************************************/

        //yield return new WaitForSeconds(period);
        yield return new WaitForSecondsRealtime(period);

        test_string = "";
        /*********************************************************************************
         *   [1]: -Y 
         *   [2]: X - PI/2 
         *   [3]: -Y
         *   [4]: X
         *   [5]: -Y + PI/2
         *   [6]: X
        **********************************************************************************/
        test_string = ((DateTime.Now.Ticks - start_tick) * 0.0001) + " "
                        + (-articulation_body_list[2].transform.localRotation.eulerAngles.y * (Math.PI / 180.0)) + " "
                        + (articulation_body_list[3].transform.localRotation.eulerAngles.x * (Math.PI / 180.0) - (Math.PI / 2)) + " "
                        + (-articulation_body_list[4].transform.localRotation.eulerAngles.y * (Math.PI / 180.0)) + " "
                        + (articulation_body_list[5].transform.localRotation.eulerAngles.x * (Math.PI / 180.0)) + " "
                        + (-articulation_body_list[6].transform.localRotation.eulerAngles.x * (Math.PI / 180.0) + (Math.PI / 2)) + " "
                        + (articulation_body_list[7].transform.localRotation.eulerAngles.x * (Math.PI / 180.0)) + " ";

        test_writer.WriteLine(test_string);
    }

    public void OnPointerClick(PointerEventData eventData)
    {
        if (boolean_button != true)
        {
            test_init();
        }
        else
        {
            boolean_button = false;

            //wait for a moment and then exit 
            StartCoroutine(delay_close());

            button_material.color = button_material.color + color_offset - color_offset_2;
        }
    }

    IEnumerator delay_close()
    {
        yield return new WaitForSecondsRealtime(period*2);
        test_writer.Close();
    }
}
